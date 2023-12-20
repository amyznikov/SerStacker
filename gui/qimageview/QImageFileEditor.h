/*
 * QImageFileEditor.h
 *
 *  Created on: Feb 24, 2021
 *      Author: amyznikov
 */

#ifndef __QImageFileEditor_h__
#define __QImageFileEditor_h__

#include "QImageEditor.h"
#include <gui/qplaysequencecontrol/QPlaySequenceControl.h>
#include <core/io/c_input_sequence.h>

class QImageFileEditor :
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QImageFileEditor ThisClass;
  typedef QImageEditor Base;

  QImageFileEditor(QWidget * parent = nullptr);
  QImageFileEditor(QImageScene * scene, QWidget * parent = nullptr);

  void openImage(const std::string & pathfilename);
  void openImage(const QString & pathfilename);
  void openImages(const std::vector<std::string> & pathfilenames);
  void openImages(const QStringList & pathfilenames);
  void setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/) override;
  void editImage(cv::InputArray image, cv::InputArray mask, bool make_copy = false) override;

  void closeCurrentSequence();

  const c_input_sequence::sptr & input_sequence() const;

  void setDebayerAlgorithm(DEBAYER_ALGORITHM algo);
  DEBAYER_ALGORITHM debayerAlgorithm() const;

  void setDropBadPixels(bool v);
  bool dropBadPixels() const;

  void setBadPixelsVariationThreshold(double v);
  double badPixelsVariationThreshold() const;

Q_SIGNALS:
  void onInputImageLoad(const cv::Mat & image, const cv::Mat & mask, COLORID colorid, int bpp);
  void debayerAlgorithmChanged();
  void dropBadPixelsChanged();
  void badPixelsVariationThresholdChanged();

protected Q_SLOTS:
  void startDisplay();
  void loadNextFrame();
  void onSeek(int pos);

protected:
  //void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  c_input_sequence::sptr input_sequence_;
  QPlaySequenceControl * playControls = nullptr;
  DEBAYER_ALGORITHM debayerAlgorithm_ = DEBAYER_DEFAULT;
  bool filterBadPixels_ = false;
  double badPixelsVariationThreshold_ = 5;

  VLO_DATA_CHANNEL vlo_data_channel_ =
      VLO_DATA_CHANNEL_AMBIENT;


};

#endif /* __QImageFileEditor_h__ */
