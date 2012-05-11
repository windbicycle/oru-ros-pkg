#include "../include/object_loader.h"
#include "../include/debug.h"
#include <math.h>
#include <assert.h>

namespace ICR
{
std::vector<double*> points_buffer;
std::vector<double*> normals_buffer;
TargetObject* callback_obj_ptr=NULL;
bool face_callback_run=false;
Eigen::VectorXi map_normal2vertex;
//--------------------------------------------------------------------
//--------------------------------------------------------------------
ObjectLoader::ObjectLoader() : object_(new TargetObject()),object_loaded_(false){}
//--------------------------------------------------------------------
ObjectLoader::ObjectLoader(ObjectLoader const& src) : object_(src.object_), object_loaded_(src.object_loaded_) {}
//--------------------------------------------------------------------
ObjectLoader& ObjectLoader::operator=(ObjectLoader const& src)
{
  if (this !=&src)
    {
      object_=src.object_;
      object_loaded_=src.object_loaded_;
    }
  return *this;
}

std::ostream& operator<<(std::ostream& stream, ObjectLoader const& obj_loader)
{
  stream <<'\n'<<"OBJECT LOADER: "<<'\n'
         <<"Is object loaded: "<<std::boolalpha<<obj_loader.object_loaded_<<'\n'
         <<"Loaded object name: "<<obj_loader.object_->getName()<<'\n'<<'\n';

  return stream;
}
//--------------------------------------------------------------------
ObjectLoader::~ObjectLoader(){}
//--------------------------------------------------------------------
const TargetObjectPtr ObjectLoader::getObject()const{return object_;}
//--------------------------------------------------------------------
TargetObjectPtr ObjectLoader::getObject(){return object_;}
//--------------------------------------------------------------------
bool ObjectLoader::objectLoaded()const{return object_loaded_;}
//--------------------------------------------------------------------
void ObjectLoader::loadObject(std::string const& file,std::string const& name)
{
  if(object_loaded_)
     object_.reset(new TargetObject());

  assert(points_buffer.size()==0);
  assert(normals_buffer.size()==0);
  assert(!face_callback_run);
  assert(callback_obj_ptr==NULL);

  callback_obj_ptr=object_.get();
  obj::obj_parser obj_parser;
  obj_parser.geometric_vertex_callback(geometric_vertex_callback);
  obj_parser.vertex_normal_callback(vertex_normal_callback);
  obj_parser.face_callbacks(triangular_face_geometric_vertices_callback,
                             triangular_face_geometric_vertices_texture_vertices_callback,
                             triangular_face_geometric_vertices_vertex_normals_callback,
                             triangular_face_geometric_vertices_texture_vertices_vertex_normals_callback,
                             quadrilateral_face_geometric_vertices_callback,
                             quadrilateral_face_geometric_vertices_texture_vertices_callback,
                             quadrilateral_face_geometric_vertices_vertex_normals_callback,
                             quadrilateral_face_geometric_vertices_texture_vertices_vertex_normals_callback,
                             polygonal_face_geometric_vertices_begin_callback,
                             polygonal_face_geometric_vertices_vertex_callback,
                             polygonal_face_geometric_vertices_end_callback,
                             polygonal_face_geometric_vertices_texture_vertices_begin_callback,
                             polygonal_face_geometric_vertices_texture_vertices_vertex_callback,
                             polygonal_face_geometric_vertices_texture_vertices_end_callback,
                             polygonal_face_geometric_vertices_vertex_normals_begin_callback,
                             polygonal_face_geometric_vertices_vertex_normals_vertex_callback,
                             polygonal_face_geometric_vertices_vertex_normals_end_callback,
                             polygonal_face_geometric_vertices_texture_vertices_vertex_normals_begin_callback,
                             polygonal_face_geometric_vertices_texture_vertices_vertex_normals_vertex_callback,
                             polygonal_face_geometric_vertices_texture_vertices_vertex_normals_end_callback);
  obj_parser.parse(file);
 
  if(points_buffer.size() < normals_buffer.size())
    {
      std::cout<<"Error in ObjectLoader - Provide an .obj file with non-smoothed vertex normals. The mapping from vertices to vertex normals has to be unique. Exiting..." <<std::endl;
      exit(1);
    }

  object_->num_cp_=points_buffer.size();
  //after the triangular_face_geometric_vertices_vertex_normals_callback, the neighbors_ lists of the contact points 
  //can contain duplicate point ids which are filtered below. This is faster than to call sort() & unique() in the callback itself;
  for(uint cp_id=0; cp_id < object_->num_cp_; cp_id++)
    {
        object_->contact_points_[cp_id]->filterDuplicateNeighbors();
        delete points_buffer[cp_id];
    }
  for(uint vn_id=0; vn_id < normals_buffer.size(); vn_id++)
    delete normals_buffer[vn_id];
 
  object_->name_=name;
  object_loaded_=true;

  points_buffer.clear();
  normals_buffer.clear();
  callback_obj_ptr=NULL;
  face_callback_run=false;

#ifdef DEBUG_OBJECT_LOADER //write the loaded object
  remove("../debug/points.txt");
  remove("../debug/normals.txt");
  remove("../debug/neighbors.txt");
  FILE* cp=fopen ("../debug/points.txt","a");
  FILE* vn=fopen ("../debug/normals.txt","a");
  FILE* nb=fopen ("../debug/neighbors.txt","a");
  if(!vn |!cp |!nb )
    {
      std::cout<<"Error in ObjectLoader: Couldn't write to file. Exiting..."<<std::endl;
      exit(1);
    }
  Eigen::Vector3d cp_vtx;
  Eigen::Vector3d cp_vtx_normal;
  for(uint id=0; id < object_->num_cp_;id++)
    {
      cp_vtx=*object_->getContactPoint(id)->getVertex();
      cp_vtx_normal=*object_->getContactPoint(id)->getVertexNormal();
      fprintf(cp, "% f % f % f \n", cp_vtx(0),cp_vtx(1),cp_vtx(2));
      fprintf(vn, "% f % f % f \n", cp_vtx_normal(0),cp_vtx_normal(1),cp_vtx_normal(2));

      for (ConstIndexListIterator it= object_->getContactPoint(id)->getNeighborItBegin(); it !=object_->getContactPoint(id)->getNeighborItEnd(); it++)
        fprintf(nb, "% d",*it+1);//Indexing of neighboring points starting from 1 instead of 0

      fprintf(nb, "\n");
    }
  fclose (cp); fclose (vn); fclose (nb);
#endif
}
//--------------------------------------------------------------------
void ObjectLoader::geometric_vertex_callback(obj::float_type x, obj::float_type y, obj::float_type z)
{ 
  points_buffer.push_back(new double[3]{x,y,z});
}
//--------------------------------------------------------------------
void ObjectLoader::vertex_normal_callback(obj::float_type x, obj::float_type y, obj::float_type z)
{
  assert(fabs(sqrt(x*x+y*y+z*z) -1) <=EPSILON_UNIT_NORMAL);
  normals_buffer.push_back(new double[3]{x,y,z});
}
//--------------------------------------------------------------------
void ObjectLoader::triangular_face_geometric_vertices_vertex_normals_callback(const obj::index_2_tuple_type& t_1, const obj::index_2_tuple_type& t_2, const obj::index_2_tuple_type& t_3)
{

  if(!face_callback_run) //If the function is called the first time for a new object, the contact point list has to be initialized
    {
      callback_obj_ptr->contact_points_.resize(points_buffer.size(),NULL);
      map_normal2vertex.resize(points_buffer.size());
      map_normal2vertex.fill(NOT_VISITED);
    }
  face_callback_run=true;

  //get the current facet & facet normal indices - libobj indexes from 1, thus the shifting by -1; the second values of the tuples index the corresponding vertex normals;
  //The uint cast is necessary in C++0x to avoid a narrowing conversion error
  uint facet[3]={(uint)(std::tr1::get<0>(t_1)-1),(uint)(std::tr1::get<0>(t_2)-1),(uint)(std::tr1::get<0>(t_3)-1)};
  uint vertex_normals[3]={(uint)(std::tr1::get<1>(t_1)-1),(uint)(std::tr1::get<1>(t_2)-1),(uint)(std::tr1::get<1>(t_3)-1)};

  //Create and push new contact points on the corresponding list of object_ if this contact point has not been visited before. 
  for(uint i=0;i<3;i++)
    {
      if(map_normal2vertex(facet[i])==NOT_VISITED) 
	{
	  map_normal2vertex(facet[i])=(int)vertex_normals[i];
          callback_obj_ptr->contact_points_[facet[i]]=new ContactPoint(points_buffer[facet[i]],normals_buffer[vertex_normals[i]],facet[i]);
        }
      else
	if(map_normal2vertex(facet[i]) != (int)vertex_normals[i])//Just a check if the .obj file is valid
	  {
	    std::cout<<"Error in ObjectLoader - Provide an .obj file with a unique mapping from vertices to vertex normals. Exiting..." <<std::endl;
	    exit(1);
	  }

      //Pushing the neighbors of the current contact point on the corresponding list. Note, that its possible that the neighbor list of a contact point contains duplicate
      //neighbors. This is cleaned up afterwards in TargetObject::loadObject since its faster than to call a sort() & unique() here. 
      for(uint j=0; j < 3; j++) 
	if (i!=j)
	  callback_obj_ptr->contact_points_[facet[i]]->addNeighbor(facet[j]);
    }

}
//--------------------------------------------------------------------
void ObjectLoader::triangular_face_geometric_vertices_callback(obj::index_type,obj::index_type,obj::index_type )
{
 std::cout<<"\n"<<"Error in ObjectLoader - triangular_face_geometric_vertices_callback is only a dummy implementation. Provide an .obj file with vertex normals. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::triangular_face_geometric_vertices_texture_vertices_callback(const obj::index_2_tuple_type&, const obj::index_2_tuple_type& , const obj::index_2_tuple_type& )
{
 std::cout<<"\n"<<"Error in ObjectLoader - triangular_face_geometric_vertices_texture_vertices_callback is only a dummy implementation. Provide an .obj file without texture vertices. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::triangular_face_geometric_vertices_texture_vertices_vertex_normals_callback(const obj::index_3_tuple_type& , const obj::index_3_tuple_type& , const obj::index_3_tuple_type& )
{
 std::cout<<"\n"<<"Error in ObjectLoader - triangular_face_geometric_vertices_texture_vertices_vertex_normals_callback is only a dummy implementation. Provide an .obj file without texture vertices. Exiting..." <<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::quadrilateral_face_geometric_vertices_callback(obj::index_type ,obj::index_type ,obj::index_type ,obj::index_type)
{
 std::cout<<"\n"<<"Error in ObjectLoader - quadrilateral_face_geometric_vertices_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::quadrilateral_face_geometric_vertices_texture_vertices_callback(const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&)
{
 std::cout<<"\n"<<"Error in ObjectLoader - quadrilateral_face_geometric_vertices_texture_vertices_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::quadrilateral_face_geometric_vertices_vertex_normals_callback(const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type& )
{
 std::cout<<"\n"<<"Error in ObjectLoader - quadrilateral_face_geometric_vertices_vertex_normals_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::quadrilateral_face_geometric_vertices_texture_vertices_vertex_normals_callback(const obj::index_3_tuple_type& , const obj::index_3_tuple_type&, const obj::index_3_tuple_type& , const obj::index_3_tuple_type& )
{
 std::cout<<"\n"<<"Error in ObjectLoader - quadrilateral_face_geometric_vertices_texture_vertices_vertex_normals_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_begin_callback(obj::index_type ,obj::index_type,obj::index_type)
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_begin_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_vertex_callback(obj::index_type)
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_vertex_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_end_callback()
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_end_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_texture_vertices_begin_callback(const obj::index_2_tuple_type& , const obj::index_2_tuple_type&, const obj::index_2_tuple_type&)
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_texture_vertices_begin_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_texture_vertices_vertex_callback(const obj::index_2_tuple_type&)
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_texture_vertices_vertex_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_texture_vertices_end_callback()
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_texture_vertices_end_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_vertex_normals_begin_callback(const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&)
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_vertex_normals_begin_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_vertex_normals_vertex_callback(const obj::index_2_tuple_type&)
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_vertex_normals_vertex_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_vertex_normals_end_callback()
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_vertex_normals_end_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_texture_vertices_vertex_normals_begin_callback(const obj::index_3_tuple_type&, const obj::index_3_tuple_type&, const obj::index_3_tuple_type&)
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_texture_vertices_vertex_normals_begin_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_texture_vertices_vertex_normals_vertex_callback(const obj::index_3_tuple_type&)
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_texture_vertices_vertex_normals_vertex_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
void ObjectLoader::polygonal_face_geometric_vertices_texture_vertices_vertex_normals_end_callback()
{
 std::cout<<"\n"<<"Error in ObjectLoader - polygonal_face_geometric_vertices_texture_vertices_vertex_normals_end_callback is only a dummy implementation. Provide an .obj file with a triangulated mesh. Exiting..."<<std::endl;
 exit(1);
}
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR

