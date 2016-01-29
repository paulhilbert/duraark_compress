#ifndef _DURAARK_COMPRESS_BLOCK_INFO_HPP_
#define _DURAARK_COMPRESS_BLOCK_INFO_HPP_

#include "common.hpp"
#include <boost/spirit/include/qi.hpp>

namespace duraark_compress {

typedef enum class block_type_ : int { scan, ifc_element, residual, undefined } block_type_t;

std::set<uint32_t> parse_index_list_(const std::string& str) {
    using boost::spirit::qi::uint_;
    using boost::spirit::qi::char_;
    using boost::spirit::qi::phrase_parse;
    using boost::spirit::ascii::space;

    auto first = str.begin();
    auto last = str.end();
    uint32_t index_begin = 0, index_end = 0;
    std::set<uint32_t> indices;
    auto match_first = [&] (uint32_t idx) { index_begin = index_end = idx; };
    auto match_second = [&] (uint32_t idx) { index_end = idx; };
    auto match_after = [&] () { for (uint32_t i=index_begin; i<=index_end; ++i) { indices.insert(i); }; };
    bool r = phrase_parse(first, last, *((uint_[match_first] >> -(char_('-') >> uint_[match_second]))[match_after]), space);
    if (!r || first != last) {
        return std::set<uint32_t>();
    }
    return indices;
}

struct block_info {
    block_type_t type;
    std::vector<uint32_t> patch_indices;
    std::string ifc_guid;
    std::string ifc_type;

    template <typename Archive>
    void save(Archive& ar) const {
        std::string type_string;
        switch (type) {
            case block_type_t::scan: type_string = "scan"; break;
            case block_type_t::ifc_element: type_string = "ifc_element"; break;
            case block_type_t::residual: type_string = "residual"; break;
            default: type_string = "undefined"; break;
        }
        ar(cereal::make_nvp("type", type_string));
        ar(cereal::make_nvp("ifc_guid", ifc_guid));
        ar(cereal::make_nvp("ifc_type", ifc_type));

        // create index string
        std::string indices_str;
        std::vector<std::vector<uint32_t>> cons;
        for (auto idx : patch_indices) {
            if (!cons.size() || cons.back().back() != (idx - 1)) {
                cons.emplace_back(1, idx);
            } else {
                cons.back().push_back(idx);
            }
        }
        bool not_first = false;
        indices_str.clear();
        for (const auto& c : cons) {
            if (!c.size()) continue;
            if (not_first++) indices_str += " ";
            indices_str += std::to_string(c[0]);
            if (c.size() > 2) indices_str += "-" + std::to_string(c.back());
        }
        ar(cereal::make_nvp("patch_indices", indices_str));

    }

    template <typename Archive>
    void load(Archive& ar) {
        std::string type_string;
        ar(cereal::make_nvp("type", type_string));
        ar(cereal::make_nvp("ifc_guid", ifc_guid));
        ar(cereal::make_nvp("ifc_type", ifc_type));

        std::string indices_str;
        ar(cereal::make_nvp("patch_indices", indices_str));
        std::set<uint32_t> iset = parse_index_list_(indices_str);
        patch_indices = std::vector<uint32_t>(iset.begin(), iset.end());

        if (type_string == "scan") {
            type = block_type_t::scan;
        } else if (type_string == "ifc_element") {
            type = block_type_t::ifc_element;
        } else if (type_string == "residual") {
            type = block_type_t::residual;
        } else {
            type = block_type_t::undefined;
        }
    }
};

} // duraark_compress

#endif /* _DURAARK_COMPRESS_BLOCK_INFO_HPP_ */
