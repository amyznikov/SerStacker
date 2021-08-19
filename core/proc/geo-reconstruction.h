/*
 * geo-reconstruction.h
 *
 *  Created on: Jul 22, 2019
 *      Author: amyznikov
 *
 *  Morphological reconstruction with dilation and erosion using hybrid algorithm
 *
 *  Translated from MorphoLibJ https://imagej.net/MorphoLibJ
 *  https://github.com/ijpb/MorphoLibJ/blob/master/src/main/java/inra/ijpb/morphology/geodrec/GeodesicReconstructionHybrid.java
 */

#ifndef __geo_reconstruction_h__
#define __geo_reconstruction_h__

#include <opencv2/opencv.hpp>

//! @addtogroup morphology
//! @{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Geodesic Dilation (δg(f))
///  https://clouard.users.greyc.fr/Pantheon/experiments/morphology/index-en.html#ch2-B
///
/// A geodesic dilation is a conditional dilation.
/// An additional mask image is used as a limit to the propagation of the dilation.
///    δg(marker_image(x,y)) = δ(marker_image(x,y)) ∧ mask_image(x,y)
///
///  Effects:
///    * The brighter features of marker_image are dilated only over the areas specified by the
//       mask image  (ie, pixels > 0 in the mask image).
///
/// @params
///   marker_image - image to be dilated (e.g. seed points)
///   ,ask_image - image limitimg dilation (e.g. mask liming dilation)
///
///  The points of marker_image are dilated only if they are within of the mask
///
int geo_dilate(cv::InputArray marker_image, cv::InputArray mask_image, cv::Mat & dst,
    cv::InputArray SE, const cv::Point & anchor = cv::Point(-1, -1), int max_iterations = -1,
    int borderType = cv::BORDER_CONSTANT, const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

int geo_dilate(cv::InputArray marker_image, cv::InputArray mask_image, cv::Mat & dst,
    int connectivity = 8, int max_iterations = -1, int borderType = cv::BORDER_CONSTANT,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());








///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Geodesic Erosion (εg(f))
///  https://clouard.users.greyc.fr/Pantheon/experiments/morphology/index-en.html#ch2-B
///
/// A geodesic erosion is a conditional erosion.
/// An additional mask image g(x,y) is used as a limit to the propagation of the erosion.
///
///  εg(f(x,y)) = ε(f(x,y))C ∧ g(x,y)
///
/// The geodesic erosion used 3 parameters.
/// The first two ones defines the structuring element, whereas the last one specifies the number of iterations.
///
/// Effect
///    * The dark features of marker_image are eroded only over the areas specified by the mask image (ie, pixels > 0 in the mask image).
///
int geo_erode(cv::InputArray marker_image, cv::InputArray mask_image, cv::Mat & dst,
    cv::InputArray SE, const cv::Point & anchor = cv::Point(-1,-1), int max_iterations = -1,
    int borderType = cv::BORDER_CONSTANT, const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());

int geo_erode(cv::InputArray marker_image, cv::InputArray mask_image, cv::Mat & dst,
    int connectivity = 8, int max_iterations = -1, int borderType = cv::BORDER_CONSTANT,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// @brief Geodesic reconstruction by dilation using hybrid algorithm.
///
/// The reconstruction by dilation is similar to the geodesic dilation applied until idempotence
/// (ie., until the resulting image at the iteration i+1 equals the resulting image at iteration i).
/// However, the mask image does not act as a mask but to force the dilation to remain above the mask image.
/// Thus, the mask image must be greater than or equal to the marker image.
/// With reconstruction by dilation, the structuring element is no longer a parameter of the reconstruction.
/// Only the connectivity (4 or 8) is used.
//
/// Effects
///    * Removes dark features smaller than the structuring element, without altering the shape.
///    * Reconstructs connected components of reference_image (mask_image) from the preserved features in marker_image.
//
/// More details see in:
///   "Digital image processing using matlab" by Rafael C. Gonzales.
///   10.5 Morphological Reconstruction
///  http://www.cs.tau.ac.il/~turkel/notes/segmentation_morphology.pdf
//
/// See also:
///  https://clouard.users.greyc.fr/Pantheon/experiments/morphology/index-en.html#ch2-B
//
/// This implementation is based on java version from ImageJ MorphoLibJ:
///  https://imagej.net/MorphoLibJ
///  https://github.com/ijpb/MorphoLibJ/blob/master/src/main/java/inra/ijpb/morphology/geodrec/GeodesicReconstructionHybrid.java
bool geo_reconstruction_dilate(cv::InputArray marker_image,
    cv::InputArray mask_image,
    /* out */ cv::Mat & reconstructed_image,
    int connectivity);




/// @brief Geodesic reconstruction by erosion using hybrid algorithm.
///
/// The reconstruction by erosion is similar to the geodesic erosion applied until idempotence
/// (ie., until the resulting image at the iteration i+1 equals the resulting image at iteration i).
/// However, the mask image is used in the reconstruction does not act as a mask but to force the erosion to remain below the mask image.
/// Thus, the mask image must be lower than or equal to the marker image. With reconstruction by erosion,
/// the structuring element is no longer a parameter of the reconstruction.
/// Only the connectivity (4 or 8) is used.
///
/// Effects
///    * Removes brighter features smaller than the structuring element, without altering the shape.
///    * Reconstructs connected components from the preserved features.
///
/// More details see in:
///   "Digital image processing using matlab" by Rafael C. Gonzales.
///   10.5 Morphological Reconstruction
///  http://www.cs.tau.ac.il/~turkel/notes/segmentation_morphology.pdf
//
/// See also:
///  https://clouard.users.greyc.fr/Pantheon/experiments/morphology/index-en.html#ch2-B
//
/// This implementation is based on java version from ImageJ MorphoLibJ:
///  https://imagej.net/MorphoLibJ
///  https://github.com/ijpb/MorphoLibJ/blob/master/src/main/java/inra/ijpb/morphology/geodrec/GeodesicReconstructionHybrid.java
bool geo_reconstruction_erode(cv::InputArray marker_image,
    cv::InputArray mask_image,
    /* out */ cv::Mat & reconstructed_image,
    int connectivity);




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief
/// Geodesic Opening by Reconstruction
///  https://clouard.users.greyc.fr/Pantheon/experiments/morphology/index-en.html#ch3-A
///
/// In the geodesic opening operation, the dilation is replaced by a reconstruction by dilatation
/// in order to recover the initial shape of the eroded objects.
/// i.e.
///    opening(F, SE) = geodilate(cv::erode(F, SE), F);
///
/// Effects
///    * Opens black holes inside white regions and separates touching white regions.
///    * Removes capes, isthmus and islands smaller than the structuring element.
///    * Grayscale and color image: removes dark features smaller than the structuring element.
///
bool geo_open(cv::InputArray src,  cv::Mat & dst, cv::InputArray SE, int connectivity = 8,
    const cv::Point & anchor = cv::Point(-1,-1), int borderType = cv::BORDER_CONSTANT,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue());



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief
/// Geodesic Closing by Reconstruction
///  https://clouard.users.greyc.fr/Pantheon/experiments/morphology/index-en.html#ch3-A
///
/// In the geodesic closing operation, the erosion is replaced by a reconstruction by erosion
/// in order to recover the initial shape of the dilated objects.
/// i.e.
///    close(F, SE) = geoerode(cv::dilate(F, SE), F);
///
/// Effects
///    * Closes black holes inside white regions and merges close white regions.
///    * Fills gulfs, channels and lakes smaller than the structuring element.
///    * Grayscale and color image: removes dark features smaller than the structuring element.
///
bool geo_close(cv::InputArray src,  cv::Mat & dst, cv::InputArray SE, int connectivity = 8,
    const cv::Point & anchor = cv::Point(-1,-1), int borderType = cv::BORDER_CONSTANT,
    const cv::Scalar & borderValue = cv::morphologyDefaultBorderValue());



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Geodesic Hole Filling
/// Fill holes in image using morphological reconstruction by dilation.
/// A hole is assumed to be a dark region which is not connected to the image border
/// and is surrounded by brighter pixels.
///
/// "Digital image processing using matlab" by Rafael C. Gonzales.
///  10.5.2  Filling Holes
///
bool geo_fill_holes(cv::InputArray src, cv::Mat & dst, int connectivity = 8);



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Geodesic Top Hat
inline void geo_tophat(const cv::Mat & src, cv::Mat & dst, cv::InputArray SE, int connectvity,
    const cv::Point & anchor = cv::Point(-1,-1),  int borderType = cv::BORDER_CONSTANT,
    const cv::Scalar& borderValue = cv::morphologyDefaultBorderValue())
{
  cv::Mat O;
  geo_open(src, O, SE, connectvity, anchor, borderType, borderValue);
  cv::subtract(src, O, dst);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief
/// Use of geo_reconstruction_dilate() to get objects that touch the borders of an image.
///
/// "Digital image processing using matlab" by Rafael C. Gonzales.
/// 10.5.3  Clearing Border Objects
bool geo_get_components_touching_borders(cv::InputArray src, cv::Mat & dst, int connectivity = 8,
    double * optional_out_minval = nullptr);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief
/// Use of geo_reconstruction_dilate() to remove objects that touch the borders of an image.
///
/// "Digital image processing using matlab" by Rafael C. Gonzales.
/// 10.5.3  Clearing Border Objects
bool geo_remove_components_touching_borders(cv::InputArray src, cv::Mat & dst, int connectivity = 8,
    double * optional_out_minval = nullptr);

//! @} morphology

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif /* __geo_reconstruction_h__ */
