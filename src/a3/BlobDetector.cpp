#include "BlobDetector.hpp"
#include "CoordinateConverter.hpp"
#include "math/angle_functions.hpp"

namespace BlobDetector {

///////////////////////////
// "PRIVATE" FUNCTIONS
///////////////////////////

// for use in findBlobs
struct BlobCell {
	bool blobColored;
	bool partOfBlob;
};

// for use in findBlobs
std::vector<Blob> 
findBlobsFromMatrix(Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic>& mat, 
	size_t minPixels);

/**
 * @brief takes an image and turns it into a matrix of BlobCells
 * that can be processed later
 */
Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic> 
imageToMatrix(image_u32_t* im, const std::array<float, 6>& hsvThresh);

Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic> 
imageToMatrix(image_u32_t* im, bool (*fn)(uint32_t));

bool isCircular(const std::vector<std::array<int, 2>>& points,
	const std::array<int, 2>& centroid);

/**
 * @brief return vector of all (x, y) coordinates related to a blob
 * @details will modify mat
 * 
 * @param mat Matrix containing info for blobs
 * @param (x,y) location to start expanding the blob. it will determine blob type
 */
std::vector<std::array<int, 2>>	 
findAndMarkBlob(Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic>& mat, 
	int x, int y);
/**
 * @brief finds centroid of vector (x, y) points
 */
std::array<int, 2> findCentroid(std::vector<std::array<int, 2>>& points);

int findAvgVal(const std::vector<std::array<int, 2>>& points,
	image_u32_t* im);


std::vector<Blob> 
findBlobs(image_u32_t* im, 
	const std::array<float, 6>& hsvThresh, size_t minPixels) {
	// turn image into matrix of BlobCells
	Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic> mat = imageToMatrix(im, hsvThresh);

	return findBlobsFromMatrix(mat, minPixels);
}

std::vector<Blob> findBlobs(image_u32_t* im,
	bool (*fn)(uint32_t), size_t minPixels) {

	Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic> mat = imageToMatrix(im, fn);

	return findBlobsFromMatrix(mat, minPixels);
}

std::vector<Blob> 
findBlobsFromMatrix(Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic>& mat, 
	size_t minPixels) {
	std::vector<Blob> ret;
	for (int row = 0; row < mat.rows(); ++row) {
		for (int col = 0; col < mat.cols(); ++col) {
			BlobCell cell = mat(row, col);
			if (cell.blobColored && !cell.partOfBlob) {
				std::vector<std::array<int, 2>> currBlob = findAndMarkBlob(mat, col, row);
				if (currBlob.size() < minPixels) {
					continue;
				}
				std::array<int, 2> center = findCentroid(currBlob);
				if (!isCircular(currBlob, center)) {
					continue;
				}
				ret.push_back({center[0], center[1], (int)currBlob.size()});
			}
		}
	}
	return ret;
}

Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic> 
imageToMatrix(image_u32_t* im, const std::array<float, 6>& hsvThresh) {
	Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic> ret(im->height, im->width);

	for (int row = 0; row < im->width; ++row) {
		for (int col = 0; col < im->height; ++col) {
			uint32_t val = im->buf[row * im->stride + col];
			std::array<uint8_t, 3> rgb = CoordinateConverter::imageValToRgb(val);
			std::array<float, 3> hsv = CoordinateConverter::rgbToHsv(rgb);
			bool blobColored = false;
			if (hsv[0] >= hsvThresh[0] && hsv[0] < hsvThresh[1] &&
				hsv[1] >= hsvThresh[2] && hsv[1] < hsvThresh[3] &&
				eecs467::angle_between(hsvThresh[4], hsvThresh[5], hsv[2])) {
				blobColored = true;
			}
			ret(row, col) = {blobColored, false};
		}
	}

	return ret;
}

Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic> 
imageToMatrix(image_u32_t* im, bool (*fn)(uint32_t)) {
	Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic> ret(im->height, im->width);

	for (int row = 0; row < im->width; ++row) {
		for (int col = 0; col < im->height; ++col) {
			uint32_t val = im->buf[row * im->stride + col];
			bool blobColored = false;
			if (fn(val)) {
				blobColored = true;
			}
			ret(row, col) = {blobColored, false};
		}
	}

	return ret;
}

std::vector<std::array<int, 2>> 
findAndMarkBlob(Eigen::Matrix<BlobCell, Eigen::Dynamic, Eigen::Dynamic>& mat, 
	int x, int y) {
	std::vector<std::array<int, 2>> ret;
	std::vector<std::array<int, 2>> toBeProcessed;
	mat(y, x).partOfBlob = true;
	toBeProcessed.push_back({{x, y}});

	while (!toBeProcessed.empty()) {
		std::array<int, 2> curr = toBeProcessed.back();
		toBeProcessed.pop_back();
		ret.push_back(curr);

		for (int row = -1; row <= 1; ++row) {
			for (int col = -1; col <= 1; ++col) {
				if (curr[1] + row >= 0 &&
					curr[1] + row < mat.rows() &&
					curr[0] + col >= 0 &&
					curr[0] + col < mat.cols()) {

					BlobCell& currCell = mat(curr[1] + row, curr[0] + col);
					if (currCell.blobColored && !currCell.partOfBlob) {
						currCell.partOfBlob = true;
						toBeProcessed.push_back({{curr[0] + col, curr[1] + row}});
					}
				}
			}
		}
	}
	return ret;
}

std::array<int, 2> findCentroid(std::vector<std::array<int, 2>>& points) {
	std::array<int, 2> ret{{0, 0}};
	for (const auto& point : points) {
		ret[0] += point[0];
		ret[1] += point[1];
	}
	ret[0] /= points.size();
	ret[1] /= points.size();

	return ret;
}

int findAvgVal(const std::vector<std::array<int, 2>>& points,
	image_u32_t* im) {

	int64_t sum = 0;
	for (const auto& pt : points) {
		sum += im->buf[im->stride * pt[1] + pt[0]];
	}
	return sum / points.size();
}


bool isCircular(const std::vector<std::array<int, 2>>& points, 
	const std::array<int, 2>& centroid) {
	int xVar = 0;
	int yVar = 0;
	for (const auto& point : points) {
		xVar += (point[0] - centroid[0]) * (point[0] - centroid[0]);
		yVar += (point[1] - centroid[1]) * (point[1] - centroid[1]);
	}
	float ratio = (float) xVar / yVar;
	// printf("ratio: %f\n", ratio);
	return std::fabs(ratio - 0.5) < 0.2;
}

}
