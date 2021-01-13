#include <iostream>
#include <string>
#include <fstream>
#include <ctime>
#include <stdlib.h>
#include "Planarity.h"
#include "PlanarSegmentation.h"
#include "StructureGraph.h"
#include "Simplification.h"
#include "FileWritter.h"

#include <chrono>

#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Surface_mesh.h>

#include <boost/program_options.hpp>
using namespace std;
namespace po = boost::program_options;

#include <tinyply.h>
using namespace tinyply;

po::options_description initOptions(){

    po::options_description options("\nOPTIONS");
    options.add_options()
            ("help,h", "Help message")
            ("working_dir,w", po::value<string>()->required(), "Working directory.\nAll paths will be treated relative to this directory.")
            ("input_file,i", po::value<string>()->required(), "Input file")
            ("output_file,o", po::value<string>(), "Output file")
            ("distance", po::value<double>(), "Distance threshold")
            ("importance", po::value<double>(), "Importance threshold")
        ;

    return options;

}

int parse(int argc, char* argv[], po::variables_map& vm){

    try{


        auto input_options = initOptions();

        // parse all options to a single options map called vm
        po::store(po::parse_command_line(argc, argv, input_options), vm);

        if (vm.count("help")){
            cout << input_options << "\n";
            return 0;
        }

        // There must be an easy way to handle the relationship between the
        // option "help" and "host"-"port"-"config"
        // Yes, the magic is putting the po::notify after "help" option check
        po::notify(vm);
    }
    catch(std::exception& e){
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    catch(...){
        std::cerr << "Unknown error when parsing command line options!" << "\n";
        return 1;
    }

    return 0;
}





int importMeshPLY(const string filepath, Mesh& mesh){

    cout << "Read mesh from " << filepath << endl;

    std::unique_ptr<std::istream> file_stream;
    // Because most people have their own mesh types, tinyply treats parsed data as structured/typed byte buffers.
    // See examples below on how to marry your own application-specific data structures with this one.
    std::shared_ptr<PlyData> vertices, faces;

    try
    {

        file_stream.reset(new std::ifstream(filepath, std::ios::binary));


        if (!file_stream || file_stream->fail()) throw std::runtime_error("file_stream failed to open " + filepath);

        PlyFile file;
        file.parse_header(*file_stream);

        // The header information can be used to programmatically extract properties on elements
        // known to exist in the header prior to reading the data. For brevity of this sample, properties
        // like vertex position are hard-coded:
        try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        // Providing a list size hint (the last argument) is a 2x performance improvement. If you have
        // arbitrary ply files, it is best to leave this 0.
        try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
        catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

        file.read(*file_stream);

    }
    catch (const std::exception & e)
    {
        std::cerr << "Caught tinyply exception: " << e.what() << std::endl;
    }



    struct float3 { float x, y, z; };
    struct uint3 { uint32_t a, b, c; };
    // TODO: variable-length branch of tinyply supports variable length polygons


    const size_t numVerticesBytes = vertices->buffer.size_bytes();
    std::vector<float3> verts(vertices->count);
    std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

    const size_t numFacetBytes = faces->buffer.size_bytes();
    std::vector<uint3> facs(faces->count);
    std::memcpy(facs.data(), faces->buffer.get(), numFacetBytes);

//    cout << "vert " << vertices->buffer.get()[0] << endl;

    vector<Point_3> points;
    vector<vector<size_t>> facets;
    // save points
    for(int i = 0; i < verts.size(); i++){
        Point_3 pt(verts[i].x, verts[i].y, verts[i].z);
        points.push_back(pt);
    }
    // save facets
    for(int i = 0; i < facs.size(); i++){
        std::vector<size_t> poly(3);
        poly[0] = facs[i].a;
        poly[1] = facs[i].b;
        poly[2] = facs[i].c;
        facets.push_back(poly);
    }


    bool oriented = CGAL::Polygon_mesh_processing::orient_polygon_soup(points, facets);
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, facets, mesh);

    return EXIT_SUCCESS;
}



int importMeshOFF(string input_file, Mesh& mesh){

    cout << "Read mesh from " << input_file << endl;

    // Read mesh
    std::ifstream input(input_file.c_str());
    if (input.fail()) {
        std::cerr << "Failed to open file \'" << input_file << "\'." << std::endl;
        return 1;
    }
    if (!(input >> mesh)) {
        std::cerr << "Failed loading model from the file." << std::endl;
        return 1;
    }
    return 0;
}

//int importMeshPLY(string input_file, Mesh& mesh){

//    cout << "Read mesh from " << input_file << endl;

//    // Read mesh
//    std::ifstream input(input_file.c_str());
//    if (input.fail()) {
//        std::cerr << "Failed to open file \'" << input_file << "\'." << std::endl;
//        return 1;
//    }
//    string comments;
//    if (!read_ply(input,mesh,comments)) {
//        std::cerr << "Failed loading model from the file." << std::endl;
//        return 1;
//    }
//    return 0;
//}




int main(int argc, char *argv[]) {
	srand(time(NULL));

    po::variables_map options;
    if(parse(argc,argv, options))
        return EXIT_FAILURE;

//    std::string input_file = "../../../data/building.off";
//    if (argc == 2)
    const string input_file = options["working_dir"].as<string>() + options["input_file"].as<string>();
    Mesh mesh;
//    if(importMeshOFF(input_file,mesh))
//        return EXIT_FAILURE;

    if(importMeshPLY(input_file,mesh))
        return EXIT_FAILURE;

    cout << "mesh faces " << mesh.number_of_faces() << endl;


    // TODO: use tinyply for importing ply meshes
	
	// Planarity inputs
	unsigned int num_rings = 3;
	/*std::cout << "Insert order of k-ring neighborhood: ";
	std::cin >> num_rings;*/

	// Segmentation inputs
	double dist_threshold = 0.0;
	VProp_geom geom = mesh.points();
	for (auto h : mesh.halfedges()) {
		auto source = geom[mesh.source(h)];
		auto target = geom[mesh.target(h)];
        dist_threshold += std::sqrt(CGAL::squared_distance(source, target));
	}
    dist_threshold /= mesh.number_of_halfedges();
//    dist_threshold *= stod(argv[2]);
    dist_threshold *= options["distance"].as<double>();
    std::cout << "Distance threshold: " << std::setprecision(2) << dist_threshold << std::endl;

	// StructureGraph inputs
//    double importance_threshold = stod(argv[3]);
    double importance_threshold = options["importance"].as<double>();
    std::cout << "Importance threshold (default: 1.0): " << std::setprecision(2) << importance_threshold << std::endl;

	// Calculate planarity
	auto start = std::chrono::steady_clock::now();
	Planarity plan;
	plan.compute(&mesh, num_rings);
	// Execution time
	auto end = std::chrono::steady_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Planarity: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// Initialize segmentation
	start = std::chrono::steady_clock::now();
	PlanarSegmentation seg;
	std::size_t seg_number = seg.apply(&mesh, dist_threshold, num_rings);
	// Execution time
	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Segmentation: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// StructureGraph
	start = std::chrono::steady_clock::now();
	StructureGraph graph;
	Graph structure_graph = graph.construct(&mesh, seg_number, importance_threshold);
	// Execution time
	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Structure Graph: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

//	// Write original mesh
//	writeMesh(&mesh, input_file + "-input.ply");
//
//	// Write graph
//	writeGraph(&mesh, &structure_graph, input_file + "-graph.obj");

	// Simplification
	start = std::chrono::steady_clock::now();
	Simplification simpl;
	Mesh simplified = simpl.apply(&mesh, &structure_graph);
	// Execution time
	end = std::chrono::steady_clock::now();
	duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	std::cout << "Simplification: " << std::setprecision(1) << duration.count() / 1000.0 << " secs" << std::endl;

	// Write simplified mesh
//	const std::string result_file = input_file + "-result.ply";
    string result_file;
    string suffix = "_d" + to_string(dist_threshold) + "_i" + to_string(importance_threshold) + ".ply";
    if(options.count("output_file"))
        result_file = options["working_dir"].as<string>() + options["output_file"].as<string>();
    else
        result_file = options["working_dir"].as<string>() + options["input_file"].as<string>() + suffix;

	writeSimplified(&simplified, result_file);
	std::cout << "Done. Result saved to file \'" << result_file << std::endl;

	return EXIT_SUCCESS;
}
