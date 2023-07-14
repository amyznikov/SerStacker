/*
 * QImageProcessingPipeline.h
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessingPipeline_h__
#define __QImageProcessingPipeline_h__

#include <gui/widgets/QSettingsWidget.h>
#include <core/pipeline/c_image_processing_pipeline.h>
#include <core/debug.h>

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

  QPipelineSettingsWidget(QWidget * parent = nullptr) :
      Base("", parent)
  {
  }

  QPipelineSettingsWidget(const QString & prefix, QWidget * parent = nullptr) :
      Base(prefix, parent)
  {
  }

  virtual QString pipelineClass() const = 0;

  virtual void setCurrentPipeline(QImageProcessingPipeline * pipeline) = 0;
  virtual QImageProcessingPipeline * currentPipeline() const = 0;

protected Q_SLOTS:
  virtual void onPipelineStateChanged() {}
};


template<class PipelineType>
class QPipelineSettingsWidgetBase:
    public QPipelineSettingsWidget
{
public:
  typedef QPipelineSettingsWidgetBase ThisClass;
  typedef QPipelineSettingsWidget Base;

  QPipelineSettingsWidgetBase(QWidget * parent = nullptr) :
      Base("", parent)
  {
  }

  QPipelineSettingsWidgetBase(const QString & prefix, QWidget * parent = nullptr) :
      Base(prefix, parent)
  {
  }

  virtual QString pipelineClass() const override
  {
    return PipelineType::class_name().c_str();
  }

  void setPipeline(PipelineType * pipeline)
  {
    CF_DEBUG("pipeline=%p", pipeline);

    if( pipeline_ ) {
      pipeline_->disconnect(this);
    }

    if( (pipeline_ = pipeline) ) {
      connect(pipeline_, &PipelineType::stateChanged,
          this, &ThisClass::onPipelineStateChanged,
          Qt::QueuedConnection);
    }

    updateControls();
  }

  PipelineType * pipeline() const
  {
    return pipeline_;
  }

  void setCurrentPipeline(QImageProcessingPipeline * pipeline) override
  {
    setPipeline(dynamic_cast<PipelineType*>(pipeline));
  }

  QImageProcessingPipeline * currentPipeline() const override
  {
    return pipeline_;
  }

protected:
  // placeholder for overrides
  virtual void update_pipeline_controls()
  {
    setEnabled(pipeline_ && !pipeline_->is_running());
  }

  void onupdatecontrols() override
  {
    if ( !pipeline_ ) {
      setEnabled(false);
    }
    else {
      Q_EMIT Base::populatecontrols();
      update_pipeline_controls();
    }
  }

  void onPipelineStateChanged() override
  {
    update_pipeline_controls();
  }


protected:
  PipelineType * pipeline_ = nullptr;
};


template<class c_pipeline>
class QImageProcessingPipelineBase :
    public QImageProcessingPipeline,
    public c_pipeline
{
public:

  typedef QImageProcessingPipelineBase ThisClass;
  typedef QImageProcessingPipeline Base;

  QImageProcessingPipelineBase(const QString & name, QObject * parent = nullptr) :
    ThisClass(name, nullptr, parent)
  {
  }

  QImageProcessingPipelineBase(const QString & name, const c_input_sequence::sptr & input_sequence, QObject * parent = nullptr) :
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
