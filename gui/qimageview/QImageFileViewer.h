/*
 * QImageFileViewer.h
 *
 *  Created on: Dec 3, 2020
 *      Author: amyznikov
 */

#ifndef __QImageFileViewer_h__
#define __QImageFileViewer_h__

#include "QImageViewer.h"
#include "QPlaySequenceControl.h"
#include <core/io/c_input_sequence.h>

class QImageFileViewer
    : public QImageViewer
{
  Q_OBJECT;
public:
  typedef QImageFileViewer ThisClass;
  typedef QImageViewer Base;

  QImageFileViewer(QWidget * parent = Q_NULLPTR);

  void openImage(const std::string & pathfilename);
  void openImage(const QString & pathfilename);
  void openImages(const std::vector<std::string> & pathfilenames);
  void openImages(const QStringList & pathfilenames);
  void setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/) override;

  void closeCurrentSequence();

  QString currentFileName() const;

  const c_input_sequence::ptr & input_sequence() const;

signals:
  void currentImageChanged();

protected slots:
  void startDisplay();
  void loadNextFrame();
  void onSeek(int pos);

protected:
  //void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  c_input_sequence::ptr input_sequence_;
  QPlaySequenceControl * playControls = Q_NULLPTR;
};

#endif /* __QImageFileViewer_h__ */
