/*
 * QImageFileEditor.cc
 *
 *  Created on: Feb 24, 2021
 *      Author: amyznikov
 */

#include "QImageFileEditor.h"
#include <gui/widgets/QWaitCursor.h>
#include <core/proc/bad_pixels.h>
#include <core/debug.h>

QImageFileEditor::QImageFileEditor(QImageScene * scene, QWidget * parent) :
  Base(scene, parent)
{
  _input_sequence = c_input_sequence::create();
  //input_sequence_->set_auto_debayer(DEBAYER_DISABLE);
  _input_sequence->set_auto_apply_color_matrix(true);

  Base::_layout->addWidget(playControls =
      new QPlaySequenceControl(this));

  connect(playControls, &QPlaySequenceControl::onSeek,
      this, &ThisClass::onSeek);
}

QImageFileEditor::QImageFileEditor(QWidget * parent) :
    ThisClass(nullptr, parent)
{
}

void QImageFileEditor::setDebayerAlgorithm(DEBAYER_ALGORITHM algo)
{
  _debayerAlgorithm = algo;
  Q_EMIT debayerAlgorithmChanged();
}

DEBAYER_ALGORITHM QImageFileEditor::debayerAlgorithm() const
{
  return _debayerAlgorithm;
}

void QImageFileEditor::setDropBadPixels(bool v)
{
  _filterBadPixels = v;
  Q_EMIT dropBadPixelsChanged();
}

bool QImageFileEditor::dropBadPixels() const
{
  return _filterBadPixels;
}

void QImageFileEditor::setBadPixelsVariationThreshold(double v)
{
  _badPixelsVariationThreshold = v;
  Q_EMIT badPixelsVariationThresholdChanged();
}

double QImageFileEditor::badPixelsVariationThreshold() const
{
  return _badPixelsVariationThreshold;
}


const c_input_sequence::sptr & QImageFileEditor::input_sequence() const
{
  return _input_sequence;
}

void QImageFileEditor::setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/)
{
  closeCurrentSequence();
  Base::setImage(image, mask, imageData, make_copy);
}

void QImageFileEditor::editImage(cv::InputArray image, cv::InputArray mask, bool make_copy)
{
  closeCurrentSequence();
  Base::editImage(image, mask, make_copy);
}

void QImageFileEditor::openImage(const std::string & pathfilename)
{
  _input_sequence->close(true);
  _input_sequence->add_source(pathfilename);
  setCurrentFileName(pathfilename.c_str());
  startDisplay();
}

void QImageFileEditor::openImage(const QString & pathfilename)
{
  openImage(pathfilename.toStdString());
}

void QImageFileEditor::openImages(const std::vector<std::string> & pathfilenames)
{
  _input_sequence->close(true);
  _input_sequence->add_sources(pathfilenames);
  setCurrentFileName(pathfilenames.empty() ? "" : pathfilenames.front().c_str());
  startDisplay();
}

void QImageFileEditor::openImages(const QStringList & pathfilenames)
{
  std::vector<std::string> std_names;

  std_names.reserve(pathfilenames.size());
  for ( const QString & s : pathfilenames ) {
    std_names.emplace_back(s.toStdString());
  }

  return openImages(std_names);
}

void QImageFileEditor::closeCurrentSequence()
{
  if( playControls->state() != QPlaySequenceControl::Stopped ) {
    playControls->setState(QPlaySequenceControl::Stopped);
  }

  playControls->hide();

  if( _input_sequence->is_open() ) {
    _input_sequence->close(true);
  }

  clear();
  setCurrentFileName("");
}


void QImageFileEditor::startDisplay()
{
  QWaitCursor wait(this);

  Base::setImage(cv::noArray(), cv::noArray(), cv::noArray(), false);

  playControls->setState(QPlaySequenceControl::Stopped);
  playControls->hide();

  if ( !_input_sequence->is_open() && !_input_sequence->open() ) {
    return;
  }

  const int num_frames = _input_sequence->size();
  if ( num_frames < 1 ) {
    QMessageBox::critical(this, "ERROR",
        "Can not determine number of frames and seek range for given source.\n"
        "Image can not be displayed correctly, seems such input source is not supported");
    return;
  }

  if( num_frames > 1 ) {
    playControls->show();
    playControls->setSeekRange(0, num_frames - 1);
    playControls->setCurpos(0);
  }

  loadNextFrame();
}

void QImageFileEditor::onSeek(int pos)
{
  if ( _input_sequence && _input_sequence->is_open() ) {
    if ( pos != _input_sequence->current_pos() ) {
      _input_sequence->seek(pos);
    }
    loadNextFrame();
  }
}

void QImageFileEditor::loadNextFrame()
{
  if ( _input_sequence && _input_sequence->is_open() ) {

    c_input_source::sptr current_source =
        _input_sequence->current_source();

    if ( current_source ) {

      QWaitCursor wait(this, current_source->size() == 1);

      _input_sequence->read(_inputImage, &_inputMask);

//      CF_DEBUG("inputImage_: %dx%d channels=%d depth=%d inputMask_: %dx%d channels=%d depth=%d",
//          inputImage_.cols, inputImage_.rows, inputImage_.channels(), inputImage_.depth(),
//          inputMask_.cols, inputMask_.rows, inputMask_.channels(), inputMask_.depth());


      Q_EMIT onInputImageLoad(_inputImage, _inputMask,
          _input_sequence->colorid(),
          _input_sequence->bpp());


      if ( _filterBadPixels && _badPixelsVariationThreshold > 0 ) {

        if( !is_bayer_pattern(_input_sequence->colorid()) ) {
          median_filter_hot_pixels(_inputImage, _badPixelsVariationThreshold, false);
        }
        else if( !extract_bayer_planes(_inputImage, _inputImage, _input_sequence->colorid()) ) {
          CF_ERROR("ERROR: extract_bayer_planes() fails");
        }
        else {
          median_filter_hot_pixels(_inputImage, _badPixelsVariationThreshold, true);
          if( !nninterpolation(_inputImage, _inputImage, _input_sequence->colorid()) ) {
            CF_ERROR("nninterpolation() fails");
          }
        }
      }
      else if( is_bayer_pattern(_input_sequence->colorid()) ) {

        debayer(_inputImage, _inputImage, _input_sequence->colorid(), _debayerAlgorithm);
        // In debayer() the DEBAYER_AVGC reduces the image size twice because of 2x2 binning
        if ( !_inputMask.empty() && _inputMask.size() != _inputImage.size() )  {
          // FIXME: INTER_NEAREST may be bad for this
          cv::resize(_inputMask, _inputMask, _inputImage.size(), 0, 0, cv::INTER_NEAREST);
        }
      }

      setCurrentFileName(current_source->filename().c_str());
      updateImage();
    }
  }
}

void QImageFileEditor::hideEvent(QHideEvent *e)
{
  closeCurrentSequence();
  Base::hideEvent(e);
}

