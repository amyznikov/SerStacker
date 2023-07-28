/*
 * QInputSourceSelection.cc
 *
 *  Created on: Jul 27, 2023
 *      Author: amyznikov
 */
#include "QInputSourceSelectionControl.h"


QInputSourceSelectionCombo::QInputSourceSelectionCombo(QWidget * parent) :
  Base(parent)
{
  setEditable(false);
  setDuplicatesEnabled(true);
  setFocusPolicy(Qt::StrongFocus);
  setSizeAdjustPolicy(QComboBox::AdjustToContents);
}

void QInputSourceSelectionCombo::setEnableExternalFile(bool v)
{
  enableExternalFile_ = true;
}

bool QInputSourceSelectionCombo::enableExternalFile() const
{
  return enableExternalFile_;
}

void QInputSourceSelectionCombo::refreshInputSources(const c_image_processing_pipeline * pipeline)
{
  Base::clear();

  if ( !pipeline ) {
    setEnabled(false);
    return;
  }

  const c_input_sequence::sptr & input_sequence =
      pipeline->input_sequence();

  if ( !input_sequence ) {
    setEnabled(false);
    return;
  }

  for ( const c_input_source::sptr & source : input_sequence->sources() ) {
    Base::addItem(QFileInfo(source->cfilename()).fileName(), QString(source->cfilename()));
  }

  if ( enableExternalFile_ ) {
    Base::addItem("Browse...");
  }
}


///////////////////////////////////////////////////////////////////////////////
