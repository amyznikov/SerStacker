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

std::string QInputSourceSelectionCombo::sourcePathFilename(int index) const
{
  return index >= 0 && index < input_sources_.size() ?
      input_sources_[index].pathfilename :
      "";
}

int QInputSourceSelectionCombo::sourceSize(int index) const
{
  return index >= 0 && index < input_sources_.size() ?
      input_sources_[index].size :
      0;
}

int QInputSourceSelectionCombo::sourceIndex(const std::string & pathfilename)
{
  for( int i = 0, n = input_sources_.size(); i < n; ++i ) {
    if( input_sources_[i].pathfilename == pathfilename ) {
      return i;
    }
  }
  return -1;
}

void QInputSourceSelectionCombo::refreshInputSources(c_image_processing_pipeline * pipeline)
{
  Base::clear();
  input_sources_.clear();

  if ( !pipeline ) {
    setEnabled(false);
    return;
  }

  const c_input_sequence::sptr & input_sequence =
      pipeline->input_sequence();

  if ( input_sequence ) {

    setEnabled(false);

    for ( const c_input_source::sptr & source : input_sequence->sources() ) {

      Base::addItem(QFileInfo(source->cfilename()).fileName());

      input_source_data source_data;
      source_data.pathfilename = source->cfilename();
      source_data.size = source->size();
      input_sources_.emplace_back(source_data);
    }
  }

  if ( enableExternalFile_ ) {
    Base::addItem("Browse...");
  }

  setEnabled(true);
}


///////////////////////////////////////////////////////////////////////////////
