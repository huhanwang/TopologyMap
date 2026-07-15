#pragma once

#include <string>
#include <vector>

#include "frenet_types.h"
#include "fused_reference/fused_reference_module.h"

namespace topology_map::algorithms {

class FrenetProjector {
public:
    FrenetProjector();
    explicit FrenetProjector(const FusedReferenceResult& reference);

    void reset(const FusedReferenceResult& reference);

    bool ok() const { return ok_; }
    const std::string& error() const { return error_; }

    bool poseAt(double s, FrenetPose* pose) const;
    FrenetPoint unproject(double s, double l) const;
    bool project(double x, double y, FrenetPoint* point) const;
    std::vector<double> sampleSValues(double spacing_m) const;

private:
    const FusedReferencePoint* pointBeforeOrAt(double s, std::size_t* index) const;

    bool ok_ = false;
    std::string error_;
    std::vector<FusedReferencePoint> points_;
};

}  // namespace topology_map::algorithms
