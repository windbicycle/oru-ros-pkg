#ifndef NDT_VISUALISATION_HH
#define NDT_VISUALISATION_HH

#include <spatial_index.h>
#include <ndt_cell.h>
#include <ndt_map.h>

#include <cstdlib>
#include <cstring>

#include <cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

#include <pangolin/pangolin.h>
#include <pangolin/simple_math.h>

#include <iostream>

namespace lslgeneric
{

////////////////////////////////////////////////
// Interface
////////////////////////////////////////////////

typedef struct NDT_VISUAL_REPRESENTATION
{
    float   loc_x,  //
            loc_y,  // mean location
            loc_z,  //

            ori_x,  //
            ori_y,  // orientation
            ori_z,  //

            sc_x,   //
            sc_y,   // scale
            sc_z,   //

            col1_R, //
            col1_G, // classification color
            col1_B, //

            col2_R, //
            col2_G, // distinct cell color
            col2_B, //

            alpha; 	// cell confidence

    bool	undoAfter; // false if current item is traversing the path; true when rendering ellipsoid
} visual_t;

/** \brief Implements a visualiser for NDT maps
 *	\details This class is used to view the contents of NDT maps in either .jff or .wrl format
 *	in a 3D graphical representation
 */
class NDTVisualiser
{

public:
    NDTVisualiser();
    NDTVisualiser(std::string filename);
    NDTVisualiser(std::string filename, std::string directory);

    void setConfigFile(std::string file)
    {
        configFile = file;
    }
    void setAOK(bool aok_)
    {
        aOK = aok_;   // try to avoid
    }

    char getFileFormat()
    {
        return fileFormat_;
    }
    bool getAOK()
    {
        return aOK;
    }
    std::string getConfigFile()
    {
        return configFile;
    }

    /**
     * \brief function for parsing and loading visual data from maps
     */
    void loadMap(std::string filename);

    /**
     * \brief function for parsing and loading visual data from a graph of maps
     */
    void loadGraph(std::string filename, std::string dir_prefs);

    /**
     * \brief function for parsing and loading visual data from JFF v0.9 files
     */
    int loadJFF(const char * filename);

    /**
     * \brief function for parsing and loading visual data from VRML V2.0 utf8 files
     */
    int loadVRML(const char * filename);

    /**
     * \brief function for parsing and loading visual data of a graph of maps from g2o files
     */
    int loadG2O(std::string filename, std::string directory);

    /**
     * \brief function for parsing and loading visual data of a graph of maps from .bag (TUM format) files
     */
    int loadTUM(std::string filename, std::string directory);

    /**
     * \brief function for displaying the loaded content
     */
    int display();

    // /**
    //  * \brief function for displaying the loaded content
    //  */
    // int displayVisualT();

    // /**
    //  * \brief function for displaying the loaded g2o file format content
    //  */
    // int displayG2O();

    // /**
    //  * \brief function for displaying loaded TUM file format content
    //  */
    // int displayTUM();

private:
    char fileFormat_;
    bool aOK;
    std::vector<visual_t> data;
    std::string configFile;
};

////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////

NDTVisualiser::NDTVisualiser()
{
    fileFormat_ = ' ';
    aOK = false;
}

NDTVisualiser::NDTVisualiser(std::string filename)
{
    aOK = true;
    loadMap(filename);
    configFile = "preferences.cfg";
}

NDTVisualiser::NDTVisualiser(std::string filename, std::string directory)
{
    aOK = true;
    loadGraph(filename, directory);
    configFile = "preferences.cfg";
}

void NDTVisualiser::loadMap(std::string filename)
{
    // aOK = true;
    const char * fileFormat = strrchr(filename.c_str(),'.');

    if(strcmp(fileFormat, ".jff") == 0)
    {
        fileFormat_ = 'j';
        if(loadJFF(filename.c_str()) < 0)
            aOK = false;
        else
            aOK = true;
    }
    else if(strcmp(fileFormat, ".wrl") == 0)
    {
        fileFormat_ = 'v';
        if(loadVRML(filename.c_str()) < 0)
            aOK = false;
        else
            aOK = true;
    }
    else if(strcmp(fileFormat, ".g2o") == 0)
    {
        std::cerr << "Error loading map\n[ ERROR ] .g2o is a graph and requires directory path to the maps\n";
        aOK = false;
    }
    else if(strcmp(fileFormat, ".bag") == 0)
    {
        std::cerr << "Error loading map\n[ ERROR ] .bag is a graph and requires directory path to the maps\n";
        aOK = false;
    }
    else
    {
        std::cerr << "Error loading map\n[ ERROR ] Invalid file format\n";
        aOK = false;
        // exit(1);
    }
}

void NDTVisualiser::loadGraph(std::string filename, std::string directory)
{

    boost::filesystem::path dirpath(directory);
    if( !boost::filesystem::is_directory(dirpath) )
    {
        std::cerr << "Error loading graph\n[ ERROR ] " << dirpath << " is not a directory\n";
        aOK = false;
    }

    const char * fileFormat = strrchr(filename.c_str(),'.');

    if(strcmp(fileFormat, ".jff") == 0)
    {
        std::cerr << "Error loading graph\n[ ERROR ] .jff is a map and not a graph\n";
        aOK = false;
    }
    else if(strcmp(fileFormat, ".wrl") == 0)
    {
        std::cerr << "Error loading graph\n[ ERROR ] .wrl is a map and not a graph\n";
        aOK = false;
    }
    else if(strcmp(fileFormat, ".g2o") == 0)
    {
        if(aOK)
        {
            fileFormat_ = 'g';
            if(loadG2O(filename, directory) < 0)
                aOK = false;
            else
                aOK = true;
        }
    }
    else if(strcmp(fileFormat, ".bag") == 0)
    {
        if(aOK)
        {
            fileFormat_ = 't';
            if(loadTUM(filename, directory) < 0)
                aOK = false;
            else
                aOK = true;
        }
    }
    else
    {
        std::cout << "Error loading graph\n[ ERROR ] Invalid file format\n";
        aOK = false;
        // exit(1);
    }
}

int NDTVisualiser::loadJFF(const char * filename)
{
    NDTMap<pcl::PointXYZ> test(new CellVector<pcl::PointXYZ>());
    NDTMap<pcl::PointXYZ> cv(new CellVector<pcl::PointXYZ>());
    NDTMap<pcl::PointXYZ> ot(new OctTree<pcl::PointXYZ>());
    NDTMap<pcl::PointXYZ> lg(new LazyGrid<pcl::PointXYZ>(0.2));
    int retval = test.loadFromJFF(filename);
    int indexType;
    if(retval == 0)
    {
        indexType = 1;
    }
    else if (retval == -2)
    {
        indexType = 2;
    }
    else if (retval == -3)
    {
        indexType = 3;
    }
    else
    {
        std::cerr << "[ ERROR ] Error 1 loading NDTMap from jff\n";
        return -1;
    }

    // std::cout << "indexType = " << indexType << std::endl;

    std::vector<NDTCell<pcl::PointXYZ>*> allCells;
    std::vector<NDTCell<pcl::PointXYZ>*>::iterator it;

    switch(indexType)
    {
    case 1:
        retval = cv.loadFromJFF(filename);
        allCells = cv.getAllCells();
        break;
    case 2:
        retval = ot.loadFromJFF(filename);
        allCells = ot.getAllCells();
        break;
    case 3:
        retval = lg.loadFromJFF(filename);
        allCells = lg.getAllCells();
        break;
    }
    if(retval < 0)
    {
        std::cerr << "[ ERROR ] Error 2 loading NDTMap from jff\n";
        return -1;
    }

    for ( it = allCells.begin() ; it < allCells.end(); it++ )
    {
        visual_t temp;
        Eigen::Vector3d mean = (*it)->getMean();
        temp.loc_x = mean(0);
        temp.loc_y = mean(1);
        temp.loc_z = mean(2);

        //opposite order to get local transforms
        Eigen::Vector3d ori = (*it)->getEvecs().eulerAngles(2,1,0);

        // if(isnan(ori(0)) || isnan(ori(1)) || isnan(ori(2))) {
        // 	cerr << "ori is not a number\n";
        // 	return -1;
        // }
        // if(isinf(ori(0)) || isinf(ori(1)) || isinf(ori(2))) {
        // 	cerr << "ori is infinite\n";
        // 	return -1;
        // }

        temp.ori_z = ori(0);
        temp.ori_y = ori(1);
        temp.ori_x = -ori(2);

        // double F_ZERO = 10e-5;
        // if (sqrt(evals_(0)) < F_ZERO || sqrt(evals_(1)) < F_ZERO || sqrt(evals_(2)) < F_ZERO) {
        // 	cerr << "sqrt(evals) too small\n";
        // 	return -1;
        // }

        Eigen::Vector3d evals = (*it)->getEvals();
        temp.sc_x = sqrt(evals(0));
        temp.sc_y = sqrt(evals(1));
        temp.sc_z = sqrt(evals(2));

        Eigen::Vector3d col;
        switch ((*it)->getClass())
        {
        case 3:
            col<<1,0,0;
            break;
        case 0:
            col<<0,1,0;
            break;
        case 1:
            col<<0,1,1;
            break;
        case 2:
            col<<0,0,1;
            break;
        default:
            col<<0,0,0;
            break;
        }

        temp.col1_R = col(0);
        temp.col1_G = col(1);
        temp.col1_B = col(2);

        (*it)->getRGB(temp.col2_R, temp.col2_G, temp.col2_B);
        temp.alpha = 1;
        //temp.alpha = (*it)->getCellConfidence();

        temp.undoAfter = true;

        data.push_back(temp);
    }

    return 0;
}

int NDTVisualiser::loadVRML(const char * filename)
{
    FILE * file = fopen(filename, "r");
    if(file == NULL)
    {
        std::cerr << "[ ERROR ] Unable to open .wrl file\n";
        return -1;
    }

    fscanf(file,"#VRML V2.0 utf8\n");

    while(!feof(file))
    {
        visual_t temp;

        fscanf(file,"Transform {\n\t");
        fscanf(file,"\
			translation %lf %lf %lf \n\t \
			Transform {\n\
			rotation 0 0 1 %lf\n\t \
			Transform {\n\
			rotation 0 1 0 %lf\n\t\t \
			Transform { \n\
	  		rotation 1 0 0 %lf\n\t\t \
	  		Transform { \n\
	  		scale %lf %lf %lf \
	  		\n\t\tchildren [\n\t\tShape{\n\t\t\tgeometry Sphere {radius 1}\n",
               &(temp.loc_x),&(temp.loc_y),&(temp.loc_z),
               &(temp.ori_z),&(temp.ori_y),&(temp.ori_x),
               &(temp.sc_x), &(temp.sc_y), &(temp.sc_z)
              );
        fscanf(file,"\t\t\tappearance Appearance {\n\t\t\tmaterial Material \
	  		{ diffuseColor %lf %lf %lf }\n}\n}\n]\n}}}}}\n",
               &(temp.col1_R),&(temp.col1_G),&(temp.col1_B)
              );

        temp.col2_R = 1-temp.col1_R;
        temp.col2_G = 1-temp.col1_G;
        temp.col2_B = 1-temp.col1_B;

        temp.alpha = 1.0;

        temp.undoAfter = true;

        data.push_back(temp);
    }

    fclose(file);
    return 0;
}

int NDTVisualiser::loadG2O(std::string filename, std::string directory)
{
    // similar as below but read type string and only read vertices (not edges)
    // every line skip id-tag and also id-nr
    float tx, ty, tz, qx, qy, qz, qw;
    visual_t temp;

    // read line by line
    std::ifstream graph_file(filename.c_str());
    std::string line;
    boost::filesystem::directory_iterator dir(directory.c_str()), it;
    it = dir;
    while(getline(graph_file, line))
    {

        // skip comments and edges (things that are not vertices)
        if(line[0] == '#' || line.substr(0,6).compare("VERTEX") != 0)
        {
            // ignore line
            continue;
        }

        // skip type info ("VERTEX_SE3:QUAT") and id
        size_t pos = line.find_first_of(" ", line.find_first_of(" ")+1)+1;

        line = line.substr(pos);
        std::istringstream line_stream(line);
        line_stream >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        // make adjustment of perspective/view (undoAfter = false)
        Eigen::Quaternion<float> quat(qw,qx,qy,qz);

        temp.loc_x = tx;
        temp.loc_y = ty;
        temp.loc_z = tz;

        //opposite order to get local transforms
        Eigen::Matrix3f rotMat(quat.toRotationMatrix());
        Eigen::Vector3f ori = rotMat.eulerAngles(2,1,0);

        temp.ori_z = ori(0);
        temp.ori_y = ori(1);
        temp.ori_x = -ori(2);

        temp.sc_x = 1.0;
        temp.sc_y = 1.0;
        temp.sc_z = 1.0;

        temp.col1_R = 0.0;
        temp.col1_G = 0.0;
        temp.col1_B = 0.0;

        temp.col2_R = 0.0;
        temp.col2_G = 0.0;
        temp.col2_B = 0.0;

        temp.alpha = 0.0;

        temp.undoAfter = false;

        data.push_back(temp);

        // load map from current file (and filter extensions)
        const boost::filesystem::path& p = *it;
        while( // is directory or invalid file format skip file
            boost::filesystem::is_directory(p) ||
            (
                p.extension().string().compare(".jff") != 0
                && p.extension().string().compare(".wrl") != 0
            ))
        {
            it++;
        }

        loadMap(p.string());

        // undo perspective adjustments
        temp.loc_x = -tx;
        temp.loc_y = -ty;
        temp.loc_z = -tz;

        temp.ori_z = -ori(0);
        temp.ori_y = -ori(1);
        temp.ori_x = ori(2);

        data.push_back(temp);

        if(!aOK)
        {
            break;
        }

        it++;
    }

    return 0;
}

int NDTVisualiser::loadTUM(std::string filename, std::string directory)
{

    float timestamp, tx, ty, tz, qx, qy, qz, qw;
    visual_t temp;

    // read line by line
    std::ifstream graph_file(filename.c_str());
    std::string line;
    boost::filesystem::directory_iterator dir(directory.c_str()), it;
    it = dir;
    while(getline(graph_file, line))
    {

        // skip comments
        if(line[0] == '#')
        {
            // ignore line
            continue;
        }
        std::istringstream line_stream(line);
        line_stream >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        // make adjustment of perspective/view (undoAfter = false)
        Eigen::Quaternion<float> quat(qw,qx,qy,qz);

        temp.loc_x = tx;
        temp.loc_y = ty;
        temp.loc_z = tz;

        //opposite order to get local transforms
        Eigen::Matrix3f rotMat(quat.toRotationMatrix());
        Eigen::Vector3f ori = rotMat.eulerAngles(2,1,0);

        temp.ori_z = ori(0);
        temp.ori_y = ori(1);
        temp.ori_x = -ori(2);

        temp.sc_x = 1.0;
        temp.sc_y = 1.0;
        temp.sc_z = 1.0;

        temp.col1_R = 0.0;
        temp.col1_G = 0.0;
        temp.col1_B = 0.0;

        temp.col2_R = 0.0;
        temp.col2_G = 0.0;
        temp.col2_B = 0.0;

        temp.alpha = 0.0;

        temp.undoAfter = false;

        data.push_back(temp);

        // load map from current file (and filter extensions)
        const boost::filesystem::path& p = *it;
        while( // is directory or invalid file format skip file
            boost::filesystem::is_directory(p) ||
            (
                p.extension().string().compare(".jff") != 0
                && p.extension().string().compare(".wrl") != 0
            ))
        {
            it++;
        }

        loadMap(p.string());

        // undo perspective adjustments
        temp.loc_x = -tx;
        temp.loc_y = -ty;
        temp.loc_z = -tz;

        temp.ori_z = -ori(0);
        temp.ori_y = -ori(1);
        temp.ori_x = ori(2);

        data.push_back(temp);

        if(!aOK)
        {
            break;
        }

        it++;
    }

    return 0;
}

// int NDTVisualiser::display(){
// 	switch(fileFormat_){
// 		case('j'): // jff format
// 			if(displayVisualT() < 0){
// 				std::cerr << "[ ERROR ] Error displaying jff\n";
// 				return -1;
// 			};
// 			break;
// 		case('v'): // vrml format
// 			if(displayVisualT() < 0){
// 				std::cerr << "[ ERROR ] Error displaying vrml\n";
// 				return -1;
// 			};
// 			break;
// 		case('g'): // g2o format
// 			if(displayG2O() < 0){
// 				std::cerr << "[ ERROR ] Error displaying g2o\n";
// 				return -1;
// 			};
// 			break;
// 		case('t'): // TUM format
// 			if(displayTUM() < 0){
// 				std::cerr << "[ ERROR ] Error displaying TUM\n";
// 				return -1;
// 			};
// 			break;
// 		default:
// 			std::cerr << "[ ERROR ] No data available\n";
// 			return -1;
// 	}

// 	return 0;
// }

void GlobalKeyHook(const std::string& example)
{
    std::cout << example << std::endl;
}

int NDTVisualiser::display()
{

    if(!aOK)
    {
        std::cerr << "[ ERROR ] Error occured while loading file.\n";
        return -1;
    }

    // Load configuration data
    pangolin::ParseVarsFile(configFile.c_str());

    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind("Main",640,480);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
        pangolin::ModelViewLookAt(-0,0,-1, 0,0,0, pangolin::AxisY)
    );

    const int UI_WIDTH = 250;

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
                            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
                            .SetHandler(new pangolin::Handler3D(s_cam));

    // Add named Panel and bind to variables beginning 'ui'
    // A Panel is just a View with a default layout and input handling
    pangolin::View& d_panel = pangolin::CreatePanel("ui")
                              .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));

    // Demonstration of how we can register a keyboard hook to alter a Var
    pangolin::RegisterKeyPressCallback( 'b', pangolin::SetVarFunctor<double>("ui.A Double", 3.5) );

    // Demonstration of how we can register a keyboard hook to trigger a method
    pangolin::RegisterKeyPressCallback( pangolin::PANGO_CTRL + 'r', boost::bind(GlobalKeyHook, "You Pushed ctrl-r!" ) );

    int solid_ = 1, colorMode = 0;
    double color_x = 0.0, color_y = 1.0, color_z = 0.0;
    double radToDeg = 57.2957795;

    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( !pangolin::ShouldQuit() )
    {
        if(pangolin::HasResized())
            pangolin::DisplayBase().ActivateScissorAndClear();

        // Safe and efficient binding of named variables.
        // Specialisations mean no conversions take place for exact types
        // and conversions between scalar types are cheap.
        static pangolin::Var<bool> toggleColorButton("ui.Toggle Color Mode",false,false);
        static pangolin::Var<std::string> colorModeDisplay("ui.Color Mode","Class Color");
        static pangolin::Var<bool> toggleSolidButton("ui.Toggle Solid Shapes",false,false);
        // static pangolin::Var<double> a_double("ui.A Double",3,0,5);
        static pangolin::Var<int> renderingQuality("ui.Number of segments per ellipsoid",6,2,20);
        // static pangolin::Var<double> a_double_log("ui.Log scale var",3,1,1E4, true);
        static pangolin::Var<bool> a_checkbox("ui.A Checkbox",false,true);
        // static pangolin::Var<int> an_int_no_input("ui.An Int No Input",2);

        if( Pushed(toggleColorButton) )
        {
            colorMode = (colorMode + 1) % 3;
            switch(colorMode)
            {
            case 0:
                colorModeDisplay = "Class Color";
                color_x = 0.0;
                color_y = 1.0;
                color_z = 0.0;
                break;
            case 1:
                colorModeDisplay = "Distinct Color";
                color_y = 1.0;
                color_y = 0.0;
                color_z = 0.0;
                break;
            case 2:
                colorModeDisplay = "White";
                color_x = 1.0;
                color_y = 1.0;
                color_z = 1.0;
                break;
            }
        }

        if( Pushed(toggleSolidButton) )
        {
            solid_ = 1-solid_;
        }

        // Overloading of Var<T> operators allows us to treat them like
        // their wrapped types, eg:
        if( a_checkbox )
            renderingQuality = 10;

        if( !colorModeDisplay.a->Get().compare("JFF") || !colorModeDisplay.a->Get().compare("jff") )
            colorModeDisplay = "made by Jan Brenstein";

        // an_int_no_input = renderingQuality;

        // Activate efficiently by object
        // (3D Handler requires depth testing to be enabled)
        d_cam.ActivateScissorAndClear(s_cam);

        // color_x = 0.0;

        glEnable(GL_DEPTH_TEST);

        // Render some stuff

        // for(int i = 0; i<8; i++){
        // 	 for(int j = 0; j<1; j++){
        // 		 glScalef(0.5,2,0.5);
        // 		 glutWireSphere(1-solid_, renderingQuality, renderingQuality);
        // 		 glutSolidSphere(solid_, renderingQuality, renderingQuality);
        // 		 glScalef(2,0.5,2);
        // 		 glTranslatef(-2,0,0);
        // 	 }
        // 	 glRotatef(45, 0.0, 1.0, 0.0);
        // 	 // glTranslatef(20,-2,0);
        // 	 // glRotatef(45, 1.0, 0.0, 0.0);
        // 	 // color_x += 0.1;
        // 	 // glColor3f(color_x,color_y,color_z);
        // }

        std::vector<visual_t>::iterator it;
        // it = data.begin();
        // for(int i = 0; i<3; i++, it++){
        for(it = data.begin(); it != data.end(); it++)
        {
            switch(colorMode)
            {
            case 0:
                glColor4f(it->col1_R, it->col1_G, it->col1_B, it->alpha);
                break;
            case 1:
                glColor4f(it->col2_R, it->col2_G, it->col2_B, it->alpha);
                break;
            case 2:
                glColor4f(1.0, 1.0, 1.0, 1.0);
                break;
            default:
                std::cerr << "[ ERROR ] fatal error in color mode while displaying\n";
                return -1;
            }

            glTranslatef(it->loc_x, it->loc_y, it->loc_z);
            glRotatef(it->ori_z * radToDeg, 0.0, 0.0, 1.0);
            glRotatef(it->ori_y * radToDeg, 0.0, 1.0, 0.0);
            glRotatef(it->ori_x * radToDeg, 1.0, 0.0, 0.0);
            glScalef(it->sc_x, it->sc_y, it->sc_z);

            if(it->undoAfter)
            {
                glutWireSphere(1-solid_, renderingQuality, renderingQuality);
                glutSolidSphere(solid_, renderingQuality, renderingQuality);

                glScalef(1/it->sc_x, 1/it->sc_y, 1/it->sc_z);
                glRotatef(-it->ori_x * radToDeg, 1.0, 0.0, 0.0);
                glRotatef(-it->ori_y * radToDeg, 0.0, 1.0, 0.0);
                glRotatef(-it->ori_z * radToDeg, 0.0, 0.0, 1.0);
                glTranslatef(-it->loc_x, -it->loc_y, -it->loc_z);
            }

        }

        // Render our UI panel when we receive input
        if(pangolin::HadInput())
            d_panel.Render();

        // Swap frames and Process Events
        pangolin::FinishGlutFrame();
    }

    return 0;
}

} // end namespace

#endif
