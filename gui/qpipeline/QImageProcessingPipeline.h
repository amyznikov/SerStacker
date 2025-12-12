/*
 * QImageProcessingPipeline.h
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessingPipeline_h__
#define __QImageProcessingPipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/pipeline/c_image_processing_pipeline_ctrl.h>
#include "QInputSourceSelectionControl.h"

class QImageProcessingPipeline;
class QPipelineSettingsWidget;

class QImageProcessingPipeline :
    public QObject
{
  Q_OBJECT;
public:
  typedef QImageProcessingPipeline ThisClass;
  typedef QObject Base;

  QImageProcessingPipeline(QObject * parent = nullptr) :
    Base(parent)
  {
  }

  virtual QString className() const = 0;
  virtual QString getName() const = 0;
  virtual void setName(const QString & name) = 0;
  virtual QPipelineSettingsWidget * createSettingsWidget(QWidget * parent = nullptr) const = 0;
  virtual bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) = 0;

Q_SIGNALS:
  void stateChanged();
  void frameProcessed();
};


class QPipelineSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QPipelineSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QPipelineSettingsWidget(QWidget * parent = nullptr);

  virtual QString pipelineClass() const = 0;
  virtual void setCurrentPipeline(QImageProcessingPipeline * pipeline) = 0;
  virtual QImageProcessingPipeline * currentPipeline() const = 0;

protected:
  void setup_controls(const std::vector<c_image_processing_pipeline_ctrl> & ctrls);
  void update_pipeline_input_sources();
  void update_control_states();

protected Q_SLOTS:
  virtual void onPipelineStateChanged() {}

protected:
  c_image_processing_pipeline * _pipeline = nullptr;
  std::map<QWidget*, std::function<bool(const c_image_processing_pipeline*)>> _bound_state_ctls;
  QList<QInputSourceSelectionControl*> _inputSourceCombos;
};


template<class QPipelineType>
class QPipelineSettingsWidgetTemplate:
    public QPipelineSettingsWidget
{
public:
  typedef QPipelineSettingsWidgetTemplate ThisClass;
  typedef QPipelineSettingsWidget Base;

  QPipelineSettingsWidgetTemplate(QWidget * parent = nullptr) :
      Base(parent)
  {
    setup_controls(QPipelineType::get_controls());
    updateControls();
  }

  virtual QString pipelineClass() const override
  {
    return QPipelineType::class_name().c_str();
  }

  void setCurrentPipeline(QImageProcessingPipeline * pipeline) override
  {
    setPipeline(dynamic_cast<QPipelineType*>(pipeline));
  }

  QImageProcessingPipeline* currentPipeline() const override
  {
    return dynamic_cast<QImageProcessingPipeline*>(_pipeline);
  }

  void setPipeline(QPipelineType * pipeline)
  {
    if( QImageProcessingPipeline *pp = dynamic_cast<QImageProcessingPipeline*>(_pipeline) ) {
      pp->disconnect(this);
    }

    if( (_pipeline = pipeline) ) {
      connect(pipeline, &QPipelineType::stateChanged,
          this, &ThisClass::onPipelineStateChanged,
          Qt::QueuedConnection);
    }

    update_pipeline_input_sources();
    updateControls();
  }

  QPipelineType * pipeline() const
  {
    return dynamic_cast<QPipelineType * >(_pipeline);
  }

protected:
  // placeholder for overrides
  void onupdatecontrols() override
  {
    if( !_pipeline ) {
      setEnabled(false);
    }
    else {
      populatecontrols();
      update_control_states();
    }
  }

  void onPipelineStateChanged() override
  {
    update_control_states();
  }
};


template<class c_pipeline>
class QImageProcessingPipelineTemplate :
    public QImageProcessingPipeline,
    public c_pipeline
{
public:

  typedef QImageProcessingPipelineTemplate ThisClass;
  typedef QImageProcessingPipeline Base;

  QImageProcessingPipelineTemplate(const QString & name, QObject * parent = nullptr) :
    ThisClass(name, nullptr, parent)
  {
  }

  QImageProcessingPipelineTemplate(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
    Base(parent),
    c_pipeline(name.toStdString(), input_sequence)
  {
  }

  QString className() const override
  {
    return c_pipeline::class_name().c_str();
  }

  void setName(const QString & name) override
  {
    c_pipeline::set_name(name.toStdString());
  }

  QString getName() const override
  {
    return c_pipeline::cname();
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    return c_pipeline::serialize(settings, save);
  }

  bool run(const c_input_sequence::sptr & input_sequence = nullptr) override
  {
    return c_pipeline::run(input_sequence);
  }

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override
  {
    return c_pipeline::get_display_image(frame, mask);
  }

protected:
  void on_state_changed() override
  {
    Q_EMIT stateChanged();
  }

  void on_frame_processed()  override
  {
    Q_EMIT frameProcessed();
  }

};

void registerPipelineClasses();


Q_DECLARE_METATYPE(c_image_processing_pipeline::sptr);

#endif /* __QImageProcessingPipeline_h__ */
