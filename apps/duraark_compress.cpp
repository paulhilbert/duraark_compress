#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <sys/fcntl.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
namespace fs = boost::filesystem;
namespace po = boost::program_options;

#include <pcl_compress/compress.hpp>
#include <pcl_compress/decompress.hpp>
#include <pcl_compress/zlib.hpp>
#include <decomposition.hpp>
using namespace duraark_compress;

#include "block_info.hpp"



int
main(int argc, char const* argv[]) {
    std::string file_in;
    std::string file_ifc;
    std::string file_reg;
    std::string file_out;
    std::string file_json;
    vec2i_t img_size;
    uint32_t blur_iters;
    int32_t max_points;
    uint32_t quality;
    uint32_t min_points;
    float angle_threshold;
    float epsilon;
    float bitmap_eps;
    float min_area;
    float prob;
    uint32_t max_octree_depth;
    float min_octree_leaf;
    float ratio;

    po::options_description desc("jpeg2000_test command line options");
    desc.add_options()("help,h", "Help message")
        ("input-cloud,i", po::value<std::string>(&file_in)->required(), "E57n input file")
        ("input-ifc,m", po::value<std::string>(&file_ifc)->default_value(""), "Optional IFCmesh input file (in conjunction with --input-reg/-r)")
        ("input-reg,r", po::value<std::string>(&file_reg)->default_value(""), "Optional registration RDF input file (in conjunction with --input-ifc/-m)")
        ("output,o", po::value<std::string>(&file_out)->required(), "Compressed output E57n file")
        ("output-json,j", po::value<std::string>(&file_json)->default_value(""), "Optional JSON metadata output file")
        ("ratio", po::value<float>(&ratio)->default_value(-1.f), "Compression ratio in [0,1] (overrides most compression parameters)")
        ("img-size,s", po::value<int>(&img_size[0])->default_value(32), "Image width and height")
        ("blur-iterations,b", po::value<uint32_t>(&blur_iters)->default_value(8), "Number of blur iterations")
        ("max-points-per-cell,m", po::value<int32_t>(&max_points)->default_value(-1), "Point count threshold for subdividing quadtree cells (Default: -1 => Use img-size * img-size).")
        ("quality,q", po::value<uint32_t>(&quality)->default_value(35), "JPEG2000 quality setting (try 35-40)")
        ("min-points", po::value<uint32_t>(&min_points)->default_value(20000), "Minimum number of points per primitive")
        ("angle-threshold", po::value<float>(&angle_threshold)->default_value(0.05f), "Maximum cosine angle deviation for primitives")
        ("dist-threshold", po::value<float>(&epsilon)->default_value(0.05f), "Maximum distance to surface deviation for primitives")
        ("bitmap-epsilon", po::value<float>(&bitmap_eps)->default_value(0.1f), "Size of primitive occupancy map pixel")
        ("min-area", po::value<float>(&min_area)->default_value(0.f), "Minimum area of accepted primitives")
        ("probability-threshold", po::value<float>(&prob)->default_value(0.001f), "Shortcut probability for the RANSAC")
        ("max-octree-depth", po::value<uint32_t>(&max_octree_depth)->default_value(6), "Maximum tree depth of octree")
        ("min-octree-leaf-size", po::value<float>(&min_octree_leaf)->default_value(0.2f), "Minimum leaf size of octree cells")
    ;

    // Check for required options.
    po::variables_map vm;
    bool optionsException = false;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        // po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (std::exception& e) {
        if (!vm.count("help")) {
            std::cout << e.what() << "\n";
        }
        optionsException = true;
    }
    if (optionsException || vm.count("help")) {
        std::cout << desc << "\n";
        return optionsException ? 1 : 0;
    }

    if (ratio >= 0.f && ratio <= 1.f) {
        blur_iters = static_cast<int>(8 + ratio * 24);
        quality = static_cast<uint32_t>(32.f + (1.f - ratio) * 8.f);
        img_size[0] = static_cast<int>(std::round(6.f - 3.f * ratio));
        img_size[0] = static_cast<int>(std::pow(2, img_size[0]));
        max_points = static_cast<int32_t>((1.f + ratio * 9.f) * img_size[0] * img_size[0]);
    }

    if ((file_ifc != "") != (file_reg != "")) {
        std::cerr << "Options --input-ifc/-m and --input-reg/-r may only be used in conjuntion. Aborting." << "\n";
        return 1;
    }

    fs::path path_in(file_in);
    if (!fs::exists(file_in)) {
        std::cerr << "Input E57n file \"" << file_in << "\" does not exist. Aborting." << "\n";
        return 1;
    }


    bool ifc_mode = file_ifc != "" && file_reg != "";
    ifc_mode = ifc_mode && fs::exists(fs::path(file_ifc)) && fs::exists(fs::path(file_reg));
    bool json = file_json != "";

    img_size[1] = img_size[0];
    if (max_points < 0) max_points = img_size[0] * img_size[1];

    prim_detect_params_t params = {
        min_points,
        angle_threshold,
        epsilon,
        bitmap_eps,
        min_area,
        prob
    };

    std::vector<block_info> blocks;
    pcl_compress::compressed_cloud_t result;
    pcl_compress::merged_global_data_t merged_gdata;
    if (ifc_mode) {
        throw std::runtime_error("IFC based compression has not been implemented yet!");
    } else {
        uint32_t scan_count = e57_pcl::get_scan_count(path_in.string());
        for (uint32_t scan_idx = 0; scan_idx < scan_count; ++scan_idx) {
            std::string guid;

            std::cout << "processing scan " << scan_idx << "..." << "\n";
            cloud_normal_t::Ptr cloud_in = e57_pcl::load_e57_scans_with_normals(
                path_in.string(), guid, true, nullptr, {scan_idx})[0];

            bbox3f_t bbox;
            for (const auto& p : cloud_in->points) {
                bbox.extend(p.getVector3fMap());
            }

            std::cout << "\tcomputing patches..." << "\n";
            decomposition_t decomp = primitive_decomposition<point_normal_t>(
                cloud_in, params, max_points, max_octree_depth, min_octree_leaf);

            std::vector<pcl_compress::patch_t> patches;
            for (const auto& subset : decomp) {
                pcl_compress::patch_t patch =
                    pcl_compress::compute_patch(cloud_in, subset, img_size, blur_iters);
                patches.push_back(patch);
            }

            uint32_t last_index = blocks.empty() ? 0 : blocks.back().patch_indices.back();
            block_info block;
            block.type = block_type_t::scan;
            block.patch_indices = std::vector<uint32_t>(patches.size(), 0);
            std::iota(block.patch_indices.begin(), block.patch_indices.end(), last_index);
            blocks.push_back(block);

            std::cout << "\tcompressing..." << "\n";
            vec3f_t scan_origin = cloud_in->sensor_origin_.head(3);
            pcl_compress::compressed_cloud_t::ptr_t cc = compress_patches(patches, quality, scan_idx, scan_origin);

            std::stringstream gcompr, gdata;
            gcompr.write((const char*)cc->global_data.data(), cc->global_data.size());
            gcompr.seekg(0);
            pcl_compress::zlib_decompress_stream(gcompr, gdata);
            gdata.seekg(0);

            pcl_compress::global_data_t parsed = pcl_compress::parse_global_data(gdata);
            merged_gdata.scan_origins.push_back(parsed.scan_origin);
            merged_gdata.scan_indices.push_back(parsed.scan_index);
            merged_gdata.patch_counts.push_back(parsed.num_patches);
            merged_gdata.bbs_o.push_back(parsed.bb_o);
            merged_gdata.bbs_b.push_back(parsed.bb_b);
            merged_gdata.point_counts.insert(merged_gdata.point_counts.end(), parsed.point_counts.begin(), parsed.point_counts.end());
            merged_gdata.origins.insert(merged_gdata.origins.end(), parsed.origins.begin(), parsed.origins.end());
            merged_gdata.bboxes.insert(merged_gdata.bboxes.end(), parsed.bboxes.begin(), parsed.bboxes.end());
            merged_gdata.bases.insert(merged_gdata.bases.end(), parsed.bases.begin(), parsed.bases.end());

            result.patch_image_data.insert(result.patch_image_data.end(), cc->patch_image_data.begin(), cc->patch_image_data.end());
        }
    }

    // compress global data
    std::stringstream gcompr;
    pcl_compress::zlib_compress_object(merged_gdata, gcompr);
    auto compr_length = gcompr.tellp();
    result.global_data.resize(compr_length);
    gcompr.seekg(0);
    gcompr.read((char*)result.global_data.data(), compr_length);

    fs::path path_out(file_out);
    fs::path p_path = path_out.parent_path();
    if (p_path.string() != "" && !fs::exists(p_path)) {
        fs::create_directories(p_path);
    }
    std::ofstream out(file_out.c_str());
    if (!out.good()) {
        std::cerr << "Unable to open file \"" << file_out << "\" for writing." << "\n";
        return 1;
    }
    {
        cereal::BinaryOutputArchive ar(out);
        ar(result);
    }
    out.close();


    if (json) {
        fs::path path_json(file_json);
        fs::path p_path = path_json.parent_path();
        if (p_path.string() != "" && !fs::exists(p_path)) {
            fs::create_directories(p_path);
        }
        std::ofstream out(file_json.c_str());
        {
            cereal::JSONOutputArchive ar(out);
            ar(cereal::make_nvp("blocks", blocks));
        }
        out.close();
    }
}
