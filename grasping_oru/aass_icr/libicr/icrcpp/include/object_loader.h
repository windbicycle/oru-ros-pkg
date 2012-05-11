#ifndef object_loader_h___
#define object_loader_h___

#include "target_object.h"
#include <iostream>
#include "obj.hpp"

namespace ICR
{
extern std::vector<double*> points_buffer;
extern std::vector<double*> normals_buffer;
extern TargetObject* callback_obj_ptr;
extern bool face_callback_run;
extern Eigen::VectorXi map_normal2vertex;
//--------------------------------------------------------------------
//--------------------------------------------------------------------
/*! 
 *  \brief Loads object models provided in the Wavefront .obj format and builds a ICR::TargetObject
 *  \details Note that the .obj models need to contain unique vertex normals for each vertex of the
 *  mesh, i.e., smoothing groups are not allowed.  This class uses libobj-0.1 for reading the .obj
 *  files and implements the necessary callbacks. Note that this class is not thread-safe, sice it
 *  uses following global variables: ICR::points_buffer (used to temporally store the mesh-vertices
 *  read by ICR::ObjectLoader::geometric_vertex_callback), ICR::normals_buffer (used to temporally
 *  store the vertex-normals read by ICR::ObjectLoader::vertex_normal_callback),
 *  ICR::face_callback_run (used by
 *  ICR::ObjectLoader::triangular_face_geometric_vertices_vertex_normals_callback),
 *  ICR::callback_obj_ptr (used by
 *  ICR::ObjectLoader::triangular_face_geometric_vertices_vertex_normals_callback in order to access
 *  the ICR::TargetObject::object_ member variable) and ICR::map_normal2vertex which is necessary to
 *  correlate the vertices to their respective normals. In .obj files, this correlation is contained
 *  in the facet definition and cannot be known a priori.
 *
 *  \warning Callbacks handling .obj files with
 *  texture information only comprise a dummy-implementation right now.
 *  
 */
class ObjectLoader
{
 private:

  TargetObjectPtr object_;
  bool object_loaded_;

  static void geometric_vertex_callback(obj::float_type x, obj::float_type y, obj::float_type z);
  static void vertex_normal_callback(obj::float_type x, obj::float_type y, obj::float_type z);
  static void triangular_face_geometric_vertices_vertex_normals_callback(const obj::index_2_tuple_type& t_1, const obj::index_2_tuple_type& t_2, const obj::index_2_tuple_type& t_3);
  static void triangular_face_geometric_vertices_callback(obj::index_type,obj::index_type,obj::index_type);
  static void triangular_face_geometric_vertices_texture_vertices_callback(const obj::index_2_tuple_type& , const obj::index_2_tuple_type&, const obj::index_2_tuple_type& );
  static void triangular_face_geometric_vertices_texture_vertices_vertex_normals_callback(const obj::index_3_tuple_type& , const obj::index_3_tuple_type&, const obj::index_3_tuple_type&);
  static void quadrilateral_face_geometric_vertices_callback(obj::index_type,obj::index_type,obj::index_type,obj::index_type);
  static void quadrilateral_face_geometric_vertices_texture_vertices_callback(const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&);
  static void quadrilateral_face_geometric_vertices_vertex_normals_callback(const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&);
  static void quadrilateral_face_geometric_vertices_texture_vertices_vertex_normals_callback(const obj::index_3_tuple_type& , const obj::index_3_tuple_type& , const obj::index_3_tuple_type& , const obj::index_3_tuple_type&);
  static void polygonal_face_geometric_vertices_begin_callback(obj::index_type,obj::index_type,obj::index_type);
  static void polygonal_face_geometric_vertices_vertex_callback(obj::index_type);
  static void polygonal_face_geometric_vertices_end_callback();
  static void polygonal_face_geometric_vertices_texture_vertices_begin_callback(const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&);
  static void polygonal_face_geometric_vertices_texture_vertices_vertex_callback(const obj::index_2_tuple_type&);
  static void polygonal_face_geometric_vertices_texture_vertices_end_callback();
  static void polygonal_face_geometric_vertices_vertex_normals_begin_callback(const obj::index_2_tuple_type&, const obj::index_2_tuple_type&, const obj::index_2_tuple_type&);
  static void polygonal_face_geometric_vertices_vertex_normals_vertex_callback(const obj::index_2_tuple_type&);
  static void polygonal_face_geometric_vertices_vertex_normals_end_callback();
  static void polygonal_face_geometric_vertices_texture_vertices_vertex_normals_begin_callback(const obj::index_3_tuple_type&, const obj::index_3_tuple_type&, const obj::index_3_tuple_type&);
  static void polygonal_face_geometric_vertices_texture_vertices_vertex_normals_vertex_callback(const obj::index_3_tuple_type&);
  static void polygonal_face_geometric_vertices_texture_vertices_vertex_normals_end_callback();

 public:

  ObjectLoader();
  ObjectLoader(ObjectLoader const& src);
  ObjectLoader& operator=(ObjectLoader const& src);
  friend std::ostream& operator<<(std::ostream& stream, ObjectLoader const& obj_loader);
  ~ObjectLoader();

  void loadObject(std::string const& file,std::string const& name);
  const TargetObjectPtr getObject()const;
  TargetObjectPtr getObject();
  bool objectLoaded()const;
};
//--------------------------------------------------------------------
//--------------------------------------------------------------------
}//namespace ICR
#endif
