// =============================================================================
//  timestamp_unwrapper.hpp
//  DW1000 40-bit counter wrap-around unwrapper (~17.2 s wrap period).
// =============================================================================
#pragma once

#include <cstdint>
#include <string>
#include <utility>

#include "ips_nodes_cpp/dw1000_constants.hpp"

namespace ips {

class TimestampUnwrapper40 {
public:
    explicit TimestampUnwrapper40(std::string name = "")
        : name_(std::move(name)) {}

    int64_t unwrap(int64_t raw_40) {
        if (!has_last_) {
            last_raw_ = raw_40;
            has_last_ = true;
            return raw_40;
        }
        // Forward wrap detection: large backward jump
        if (raw_40 < last_raw_ && (last_raw_ - raw_40) > HALF_WRAP_40) {
            wrap_count_ += 1;
        }
        last_raw_ = raw_40;
        return raw_40 + wrap_count_ * WRAP_40;
    }

    void reset() {
        has_last_   = false;
        last_raw_   = 0;
        wrap_count_ = 0;
    }

    const std::string& name() const { return name_; }

private:
    std::string name_;
    bool        has_last_   = false;
    int64_t     last_raw_   = 0;
    int64_t     wrap_count_ = 0;
};

} // namespace ips
