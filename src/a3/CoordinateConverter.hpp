#ifndef COORDINATE_CONVERTER_HPP
#define COORDINATE_CONVERTER_HPP

#include <stdint.h>
#include <array>
#include <vector>

// converts between various frames of reference and color

class CoordinateConverter {
public:
	// (r, theta)
	static std::array<float, 2> polarToCartesian(const std::array<float, 2>& arr);

	// angle in radians, (r, theta)
	static std::array<float, 2> cartesianToPolar(const std::array<float, 2>& arr);

	static std::array<int, 2> screenToImage(const std::array<float, 2>& arr,
		int imageWidth, int imageHeight);

	static std::array<float, 2> imageToScreen(const std::array<int, 2>& arr,
		int imageWidth, int imageHeight);

	static std::array<float, 3> rgbToHsv(const std::array<uint8_t, 3>& rgb);

	static std::array<uint8_t, 3> imageValToRgb(uint32_t val);
};

#endif /* COORDINATE_CONVERTER_HPP */
