/*
 * QCloudDatasetView.cc
 *
 *  Created on: Nov 20, 2023
 *      Author: amyznikov
 */

#include "QCloudSequenceView.h"

namespace cloudview {

QCloudSequenceView::QCloudSequenceView(QWidget * parent) :
  Base(parent)
{
}

void QCloudSequenceView::setInputSource(const c_cloudview_input_source::sptr & current_source)
{
  closeCurrentSource();

  current_source_ = current_source;

  startDisplay();
}

const c_cloudview_input_source::sptr & QCloudSequenceView::inputSource() const
{
  return current_source_;
}


void QCloudSequenceView::closeCurrentSource()
{
  if ( current_source_ ) {
    current_source_->close();
  }
}

void QCloudSequenceView::startDisplay()
{

}


} // namespace cloudview
