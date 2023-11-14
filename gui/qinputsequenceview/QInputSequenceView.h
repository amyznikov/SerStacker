/*
 * QInputSequenceView.h
 *
 *  Created on: Nov 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputSequenceView_h__
#define __QInputSequenceView_h__

#include <gui/qimageview/QImageEditor.h>
#include <gui/qcloudview/QCloudViewer.h>
#include <gui/qtextview/QTextFileViewer.h>
#include <gui/qplaysequencecontrol/QPlaySequenceControl.h>
#include <core/io/c_input_sequence.h>


class QInputSequenceView :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QInputSequenceView ThisClass;
  typedef QWidget Base;

  QInputSequenceView(QWidget * parent = nullptr);

  QInputSequenceView(QWidget * parent,
      QImageEditor * imageView,
      QCloudViewer * cloudView,
      QTextFileViewer * textView);

  QToolBar * toolbar() const;
  QToolBar * imageViewToolbar() const;
  QToolBar * cloudViewToolbar() const;
  QToolBar * textViewToolbar() const;
  QToolBar * rightToolbar() const;

  QImageEditor * imageView() const;
  QCloudViewer * cloudView() const;
  QTextFileViewer * textView() const;

  void setCurrentView(QWidget * w);
  QWidget * currentView() const;


  bool openFile(const QString & filename);
  bool openFile(const std::string & filename);

  const QString & currentFileName() const;
  const c_input_sequence::sptr & currentSequence() const;

  void setDebayerAlgorithm(DEBAYER_ALGORITHM algo);
  DEBAYER_ALGORITHM debayerAlgorithm() const;

  void setDropBadPixels(bool v);
  bool dropBadPixels() const;

  void setBadPixelsVariationThreshold(double v);
  double badPixelsVariationThreshold() const;

  void setSourceOutputType(c_input_source::OUTPUT_TYPE v);
  c_input_source::OUTPUT_TYPE sourceOutputType() const;

#if HAVE_VLO_FILE
  void setVloDataChannel(c_vlo_file::DATA_CHANNEL channel);
  c_vlo_file::DATA_CHANNEL vloDataChannel() const;
#endif

  // TODO: temporary hotfix for pipeline progressview
  void showImageView();

Q_SIGNALS:
  void visibilityChanged(bool visible);
  void currentFileNameChanged();
  void debayerAlgorithmChanged();
  void dropBadPixelsChanged();
  void badPixelsVariationThresholdChanged();
  void sourceOutputTypeChanged();
  void currentViewChanged();
#if HAVE_VLO_FILE
  void vloDataChannelChanged();
#endif

protected Q_SLOTS:
  void onStackedWidgetCurrentIndexChanged();
  void startDisplay();
  void loadNextFrame();
  void onSeek(int pos);

protected:
  void setCurrentToolbar(QToolBar * toolbar);
  void setCurrentFileName(const QString & filename);
  bool openInputSequence();
  void closeInputSequence();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  c_input_sequence::sptr currentSequence_;
  QString currentFileName_;

  DEBAYER_ALGORITHM debayerAlgorithm_ = DEBAYER_DEFAULT;
  bool filterBadPixels_ = false;
  double badPixelsVariationThreshold_ = 5;

  c_input_source::OUTPUT_TYPE sourceOutputType_ =
      c_input_source::OUTPUT_TYPE_IMAGE;

#if HAVE_VLO_FILE
  c_vlo_file::DATA_CHANNEL vlo_data_channel_ =
      c_vlo_file::DATA_CHANNEL_AMBIENT;
#endif

  QVBoxLayout * mainLayout_ = nullptr;
  QHBoxLayout * toolbarLayout_ = nullptr;
  QToolBar * mainToolbar_ = nullptr;
  QToolBar * imageViewToolbar_ = nullptr;
  QToolBar * cloudViewToolbar_ = nullptr;
  QToolBar * textViewToolbar_ = nullptr;
  QToolBar * rightToolbar_ = nullptr;
  QStackedWidget * stackWidget_ = nullptr;
  QImageEditor * imageView_ = nullptr;
  QCloudViewer * cloudView_ = nullptr;
  QTextFileViewer * textView_ = nullptr;
  QPlaySequenceControl * playControls_ = nullptr;

};

#endif /* __QInputSequenceView_h__ */
