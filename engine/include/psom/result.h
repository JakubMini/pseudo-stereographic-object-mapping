#pragma once
#include <functional>
#include <string>
#include <vector>

namespace psom {

// Every stage reports this. `ok` is set only after the stage's output was checked, never because
// a file was written.
struct Result {
    bool ok = true;
    std::string stage;
    std::string error;
    std::vector<std::string> warnings;

    static Result fail(std::string stage, std::string error) {
        Result r;
        r.ok = false;
        r.stage = std::move(stage);
        r.error = std::move(error);
        return r;
    }
    void warn(std::string w) { warnings.push_back(std::move(w)); }
    void absorb(const Result& other) {
        warnings.insert(warnings.end(), other.warnings.begin(), other.warnings.end());
    }
};

// stage name, fraction 0..1 of that stage
using Progress = std::function<void(const std::string&, double)>;

}  // namespace psom
