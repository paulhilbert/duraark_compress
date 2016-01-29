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
#include <pcl_compress/jbig2.hpp>
#include <pcl_compress/jpeg2000.hpp>
#include <pcl_compress/zlib.hpp>
#include <pcl_compress/types.hpp>
#include <decomposition.hpp>
using namespace duraark_compress;
using namespace pcl_compress;

#include <e57_pcl/write.hpp>

#include "block_info.hpp"


std::vector<block_info> parse_json_blocks(const std::string& file_json, bool& has_entities, bool& has_scans) {
    std::ifstream in(file_json.c_str());
    std::vector<block_info> blocks;
    {
        cereal::JSONInputArchive ar(in);
        ar(cereal::make_nvp("blocks", blocks));
    }
    in.close();

    has_entities = false;
    has_scans = false;
    for (const auto& block : blocks) {
        if (block.type == block_type_t::scan) {
            has_scans = true;
            continue;
        }
        if (block.type == block_type_t::ifc_element || block.type == block_type_t::residual) {
            has_entities = true;
            continue;
        }
    }

    return blocks;
}

std::vector<uint32_t> gather_patch_indices(const std::vector<block_info>& blocks, const std::vector<uint32_t>& subset, const std::vector<std::string>& ifc_types) {
    std::set<std::string> types(ifc_types.begin(), ifc_types.end());
    std::set<uint32_t> sub(subset.begin(), subset.end());
    bool skip_ifc = !types.size(), skip_scan = !sub.size();
    uint32_t scan_idx = 0;
    std::set<uint32_t> patches;
    for (const auto& block : blocks) {
        bool valid_block = false;
        if (block.type == block_type_t::ifc_element && (skip_ifc || types.find(block.ifc_type) != types.end())) {
            valid_block = true;
        }
        if (block.type == block_type_t::residual && (skip_ifc || types.find("residual") != types.end())) {
            valid_block = true;
        }
        if (block.type == block_type_t::scan && (skip_scan || sub.find(scan_idx++) != sub.end())) {
            valid_block = true;
        }

        if (!valid_block) continue;

        patches.insert(block.patch_indices.begin(), block.patch_indices.end());
    }

    std::vector<uint32_t> result(patches.begin(), patches.end());
    return result;
}

int
main(int argc, char const* argv[]) {
    std::string file_in;
    std::string file_json;
    std::string file_out;
    std::string scan_indices;
    std::vector<std::string> ifc_types;

    po::options_description desc("jpeg2000_test command line options");
    desc.add_options()("help,h", "Help message")
        ("input-cloud,i", po::value<std::string>(&file_in)->required(), "E57n input file")
        ("input-json,j", po::value<std::string>(&file_json)->default_value(""), "Optional JSON metadata output file")
        ("output,o", po::value<std::string>(&file_out)->required(), "Decompressed output E57n file")
        ("scan-indices,s", po::value<std::string>(&scan_indices)->default_value(""), "Indices string for scan subsets")
        ("ifc-types,t", po::value<std::vector<std::string>>(&ifc_types), "Indices string for scan subsets")
    ;
    po::positional_options_description p;
    p.add("ifc-types", -1);


    // Check for required options.
    po::variables_map vm;
    bool optionsException = false;
    try {
        po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
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

    fs::path path_in(file_in);
    if (!fs::exists(file_in)) {
        std::cerr << "Input E57c file \"" << file_in << "\" does not exist. Aborting." << "\n";
        return 1;
    }

    std::vector<uint32_t> subset;
    if (scan_indices != "") {
        std::set<uint32_t> s_subset = parse_index_list_(scan_indices);
        subset.assign(s_subset.begin(), s_subset.end());
    }
    bool json = file_json != "" && fs::exists(fs::path(file_json));

    if (!json && subset.size()) {
        std::cerr << "Scan subsets can only be specified when a JSON is supplied. Aborting." << "\n";
        return 1;
    }

    bool has_entities = false, has_scans = false;
    std::vector<block_info> blocks;
    if (json) {
        blocks = parse_json_blocks(file_json, has_entities, has_scans);
    }

    if (subset.size() && !has_scans) {
        std::cerr << "Scan indices specified but no scan blocks found in JSON file. Aborting." << "\n";
        return 1;
    }

    if (ifc_types.size() && !has_entities) {
        std::cerr << "IFC types specified but no entity blocks found in JSON file. Aborting." << "\n";
        return 1;
    }

    std::vector<uint32_t> patches = gather_patch_indices(blocks, subset, ifc_types);

    pcl_compress::compressed_cloud_t cc;
    std::cout << "Reading compressed cloud" << "\n";
    std::ifstream in(file_in.c_str());
    if (!in.good()) {
        std::cerr << "Unable to open file \"" << file_in << "\" for reading." << "\n";
        return 1;
    }
    {
        cereal::BinaryInputArchive ar(in);
        ar(cc);
    }
    in.close();

    std::cout << "Decompressing global data" << "\n";
    std::stringstream gcompr;
    gcompr.write((const char*)cc.global_data.data(), cc.global_data.size());
    gcompr.seekg(0);
    merged_global_data_t global_data = zlib_decompress_object<merged_global_data_t>(gcompr);

    std::cout << "Decompressing " << patches.size() << " patches" << "\n";
    cloud_normal_t::Ptr global_cloud(new cloud_normal_t());
    for (const auto& idx : patches) {
        chunk_t occmap = cc.patch_image_data[idx*2+0];
        chunk_t hmap = cc.patch_image_data[idx*2+1];
        patch_t patch;
        chunk_ptr_t chunk_jbig2(new chunk_t(occmap));
        chunk_ptr_t chunk_jpeg2k(new chunk_t(hmap));
        patch.origin = global_data.origins[idx];
        patch.local_bbox = global_data.bboxes[idx];
        patch.base = global_data.bases[idx];
        patch.occ_map = jbig2_decompress_chunk(chunk_jbig2);
        patch.height_map = jpeg2000_decompress_chunk(chunk_jpeg2k);
        cloud_normal_t::Ptr cloud = from_patches({patch});
        global_cloud->insert(global_cloud->end(), cloud->begin(), cloud->end());
    }
    e57_pcl::write_e57n(file_out, global_cloud, "some_GUID");
}
