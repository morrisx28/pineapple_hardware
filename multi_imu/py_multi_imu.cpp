#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "ImuSharedData.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <array>

void imuRS485Thread(std::atomic<bool>& running, ImuSharedData* imuData, const char* port);

namespace py = pybind11;

struct ImuSnapshot {
    int id;
    std::array<double, 4> quaternion;
    std::array<double, 3> rpy;
    std::array<double, 3> gyro;
    std::array<double, 3> accel;
};

class ImuRs485Worker {
public:
    ImuRs485Worker(const std::string& port) : running_(false), port_(port) {}
    ~ImuRs485Worker() { stop(); }

    void start() {
        if (running_) return;
        running_ = true;
        worker_ = std::thread(imuRS485Thread, std::ref(running_), &data_, port_.c_str());
    }

    void stop() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
    }

    ImuSnapshot snapshot() {
        std::lock_guard<std::mutex> lock(data_.mtx);
        ImuSnapshot snap;
        snap.id = data_.id;
        snap.quaternion = data_.quaternion;
        snap.rpy = data_.rpy;
        snap.gyro = data_.gyro;
        snap.accel = data_.accel;
        return snap;
    }

private:
    std::atomic<bool> running_;
    std::thread worker_;
    std::string port_;
    ImuSharedData data_;
};

PYBIND11_MODULE(pineapple_multi_imu, m) {
    py::class_<ImuSnapshot>(m, "ImuSnapshot")
        .def_readonly("id", &ImuSnapshot::id)
        .def_readonly("quaternion", &ImuSnapshot::quaternion)
        .def_readonly("rpy", &ImuSnapshot::rpy)
        .def_readonly("gyro", &ImuSnapshot::gyro)
        .def_readonly("accel", &ImuSnapshot::accel);

    py::class_<ImuRs485Worker>(m, "ImuRs485Worker")
        .def(py::init<const std::string&>())
        .def("start", &ImuRs485Worker::start)
        .def("stop", &ImuRs485Worker::stop)
        .def("snapshot", &ImuRs485Worker::snapshot);
}