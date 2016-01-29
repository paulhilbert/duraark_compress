#ifndef DURAARK_COMPRESS_RANGE_H_
#define DURAARK_COMPRESS_RANGE_H_

#include <utility>

namespace duraark_compress {

template <class Iter>
class range : public std::pair<Iter, Iter> {
public:
    range(const std::pair<Iter, Iter>& x) : std::pair<Iter, Iter>(x) {}
    virtual ~range() {}

    Iter
    begin() const {
        return this->first;
    }
    Iter
    end() const {
        return this->second;
    }
};

}  // duraark_compress

#endif /* DURAARK_COMPRESS_RANGE_H_ */
