#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <libobsensor/ObSensor.hpp>

int main()
{
  try {
    ob::Context context;
    auto device_list = context.queryDeviceList();
    const uint32_t device_count = device_list->getCount();

    if (device_count == 0U) {
      std::cerr << "No Orbbec camera detected." << std::endl;
      return EXIT_FAILURE;
    }

    if (device_count < 2U) {
      std::cerr << "Expected 2 Orbbec cameras, but found " << device_count << "." << std::endl;
    }

    const uint32_t output_count = std::min<uint32_t>(device_count, 2U);
    for (uint32_t i = 0; i < output_count; ++i) {
      auto device = device_list->getDevice(i);
      auto info = device->getDeviceInfo();
      const std::string serial = info->getSerialNumber();
      std::cout << "orbbec_camera_" << (i + 1U) << "_serial: " << serial << std::endl;
    }

    return device_count >= 2U ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const ob::Error &e) {
    std::cerr
      << "Orbbec SDK error: function=" << e.getFunction()
      << ", args=" << e.getArgs()
      << ", message=" << e.what()
      << ", type=" << e.getExceptionType()
      << std::endl;
  } catch (const std::exception &e) { 
    std::cerr << "Unexpected error: " << e.what() << std::endl;
  }

  return EXIT_FAILURE;
}
