#include <wyrm>

extern "C" {
#include <CommandLineInterface/CLIcore.h>
__attribute__((visibility( "default" ))) DATA data;
}

namespace py = pybind11;

PYBIND11_MODULE(pyCacao, m) {
  m.doc() = "cacao library module";

  m.def("processinfo_CTRLscreen", &processinfo_CTRLscreen);

  // py::class_<RTCController>(m, "CppRTC")
  //     .def(py::init())
  //     .def(
  //         "makeSH",
  //         [](RTCController& rtc, py::array_t<DataT> const& dark,
  //            py::array_t<DataT> const& flat, py::array_t<DataT> const& slopeRef,
  //            py::array_t<int> const& saNumMap, py::array_t<int> const& saPosMap,
  //            py::array_t<DataT> const& centroidingMap, DataT thresh,
  //            DataT modFac, int pixelNb, int wfsNb, int slopeNb, int subSize) {
  //           rtc.makeSH(dark.data(), flat.data(), slopeRef.data(),
  //                      saNumMap.data(), saPosMap.data(), centroidingMap.data(),
  //                      thresh, modFac, pixelNb, wfsNb, slopeNb, subSize);
  //         })
  //     .def("makeLS",
  //          [](RTCController& rtc, py::array_t<DataT> const& cmdMat, int dimX,
  //             int dimY,
  //             DataT gain) { rtc.makeLS(cmdMat.data(), dimX, dimY, gain); })
  //     .def("makeMV",
  //          [](RTCController& rtc, py::array_t<DataT> const& cmdMat,
  //             py::array_t<DataT> const& recoMat, int dimX, int dimY, DataT gain,
  //             DataT inteFac) {
  //            rtc.makeMV(cmdMat.data(), recoMat.data(), dimX, dimY, gain,
  //                       inteFac);
  //          })
  //     .def("computeSlope",
  //          [](RTCController& rtc, py::array_t<DataT>& slope,
  //             py::array_t<FrameT> const& frame) {
  //            rtc.computeSlope(slope.mutable_data(), frame.data());
  //          })
  //     .def("computeCmd",
  //          [](RTCController& rtc, py::array_t<DataT>& cmd,
  //             py::array_t<DataT> const& slope) {
  //            rtc.computeCmd(cmd.mutable_data(), slope.data());
  //          })
  //     .def("compute",
  //          [](RTCController& rtc, py::array_t<DataT>& cmd,
  //             py::array_t<FrameT> const& frame) {
  //            rtc.compute(cmd.mutable_data(), frame.data());
  //          })
  //     .def("resetController", &RTCController::resetController)
  //     .def("setSlopeRef",
  //          [](RTCController& rtc, py::array_t<DataT> const& slopeRef) {
  //            rtc.setSlopeRef(slopeRef.data());
  //          })
  //     .def("setCmdMatrix",
  //          [](RTCController& rtc, py::array_t<DataT> const& cmdMatrix) {
  //            rtc.setCmdMatrix(cmdMatrix.data());
  //          })
  //     .def("setRecoMatrix",
  //          [](RTCController& rtc, py::array_t<DataT> const& recoMatrix) {
  //            rtc.setRecoMatrix(recoMatrix.data());
  //          });


#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif
}
