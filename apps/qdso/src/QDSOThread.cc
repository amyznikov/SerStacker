/*
 * QDSOThread.cc
 *
 *  Created on: Jun 13, 2024
 *      Author: amyznikov
 */

#include "QDSOThread.h"
#include <core/debug.h>

namespace qdso {

QDSOThread::QDSOThread(QObject * parent) :
    Base(parent)
{
}


bool QDSOThread::start(c_dso_dataset_reader * dataset, c_dso_display * display)
{
  if( isRunning() ) {
    CF_ERROR("Thread is already runnuing ");
    return false;
  }

  if( !(this->dataset = dataset) ) {
    CF_ERROR("dataset pointer is NULL");
    return false;
  }

  this->display =
      display;

  stop_ = false;
  Base::start();

  return true;
}

void QDSOThread::stop()
{
  stop_ = true;
}


void QDSOThread::run()
{
  using namespace dso;
  CF_DEBUG("QDSOThread: enter");

  FullSystem::uptr dsoSystem(new FullSystem());
  dsoSystem->setPhotometricGamma(dataset->photometricGamma());
  dsoSystem->display = display;

  const int num_images =
      dataset->numImages();

  c_image_and_exposure input_image;

  for ( int i = 0; i < num_images && !stop_; ++i ) {

    if ( !dataset->getImage(i, &input_image) ) {
      CF_ERROR("dataset->getImage(%d) fails", i);
      break;
    }

    dsoSystem->addActiveFrame(input_image, i);

//    if ( display && display->needDisplayInputFrame() ) {
//      display->displayInputFrame(input_image, i);
//    }

    // Base::msleep(100);
  }

  CF_DEBUG("QDSOThread: leave");
}


} /* namespace qdso */
