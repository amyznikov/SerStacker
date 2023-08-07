/*
 * QInputSourceSelection.h
 *
 *  Created on: Jul 27, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QInputSourceSelection_h__
#define __QInputSourceSelection_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_image_processing_pipeline.h>

class QInputSourceSelectionControl
{
public:
  typedef QInputSourceSelectionControl ThisClass;

  virtual ~QInputSourceSelectionControl() = default;

  virtual void setEnableExternalFile(bool v) = 0;
  virtual bool enableExternalFile() const = 0;
  virtual void refreshInputSources(c_image_processing_pipeline * pipeline) = 0;
};

class QInputSourceSelectionCombo :
    public QComboBox,
    public QInputSourceSelectionControl
{
  // Q_OBJECT;
public:
  typedef QInputSourceSelectionCombo ThisClass;
  typedef QComboBox Base;

  QInputSourceSelectionCombo(QWidget * parent = nullptr);

  void setEnableExternalFile(bool v) override;
  bool enableExternalFile() const override;

  void refreshInputSources(c_image_processing_pipeline * pipeline) override;

  std::string sourcePathFilename(int index) const;
  int sourceSize(int index) const;
  int sourceIndex(const std::string & pathfilename);

protected:
  bool enableExternalFile_ = false;

  struct input_source_data {
    std::string pathfilename;
    int size = 0;
  };

  std::vector<input_source_data> input_sources_;

};



#endif /* __QInputSourceSelection_h__ */
