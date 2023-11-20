/*
 * QCloudSequenceView.h
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QCloudSequenceView_h__
#define __QCloudSequenceView_h__

#include <QtWidgets/QtWidgets>
#include "dataset/c_cloudview_input_source.h"

namespace cloudview {

class QCloudSequenceView :
    public QWidget
{
  Q_OBJECT;
public:
  typedef QCloudSequenceView ThisClass;
  typedef QWidget Base;

  QCloudSequenceView(QWidget * parent = nullptr);

  void setInputSource(const c_cloudview_input_source::sptr & current_source);
  const c_cloudview_input_source::sptr & inputSource() const;

protected:
  void closeCurrentSource();
  void startDisplay();

protected:
  c_cloudview_input_source::sptr current_source_;
};

} // namespace cloudview

#endif /* __QCloudSequenceView_h__ */
