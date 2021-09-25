/*
 * QImageFileEditor.h
 *
 *  Created on: Feb 24, 2021
 *      Author: amyznikov
 */

#ifndef __QImageFileEditor_h__
#define __QImageFileEditor_h__

#include "QImageEditor.h"
#include "QPlaySequenceControl.h"
#include <core/io/c_input_sequence.h>

class QImageFileEditor
    : public QImageEditor
{
  Q_OBJECT;
public:
  typedef QImageFileEditor ThisClass;
  typedef QImageEditor Base;

  QImageFileEditor(QWidget * parent = Q_NULLPTR);

  void openImage(const std::string & pathfilename);
  void openImage(const QString & pathfilename);
  void openImages(const std::vector<std::string> & pathfilenames);
  void openImages(const QStringList & pathfilenames);
  void setImage(cv::InputArray image, cv::InputArray mask, cv::InputArray imageData /*= cv::noArray()*/, bool make_copy /*= true*/) override;
  void editImage(cv::InputArray image, cv::InputArray mask) override;


  void closeCurrentSequence();
  const c_input_sequence::ptr & input_sequence() const;

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

#endif /* __QImageFileEditor_h__ */
