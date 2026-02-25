/*
 * QImageProcessingPipeline.h
 *
 *  Created on: Feb 8, 2021
 *      Author: amyznikov
 */

#ifndef __QImageProcessingPipeline_h__
#define __QImageProcessingPipeline_h__

#include <core/pipeline/c_image_processing_pipeline.h>
#include <gui/widgets/QSettingsWidgetTemplate.h>

class QImageProcessingPipeline;
class QPipelineSettingsWidget;
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
    Base(parent)
  {
  }

  virtual QString pipelineClass() const = 0;
  virtual void setCurrentPipeline(QImageProcessingPipeline * pipeline) = 0;
  virtual QImageProcessingPipeline * currentPipeline() const = 0;

protected Q_SLOTS:
  virtual void onPipelineStateChanged() {}
};

template<class c_pipeline_type>
class QPipelineSettingsWidgetTemplate:
    public QSettingsWidgetTemplate<c_pipeline_type, QPipelineSettingsWidget>
{
public:
  typedef QPipelineSettingsWidgetTemplate ThisClass;
  typedef QSettingsWidgetTemplate<c_pipeline_type, QPipelineSettingsWidget> Base;

  QPipelineSettingsWidgetTemplate(QWidget * parent = nullptr) :
      Base(parent)
  {
    setupControls(this, c_pipeline_type::getcontrols());
    ThisClass::setEnabled(false);
  }

  virtual QString pipelineClass() const final
  {
    return c_pipeline_type::class_name().c_str();
  }

  void setCurrentPipeline(QImageProcessingPipeline * pipeline) final
  {
    if( QImageProcessingPipeline *pp = dynamic_cast<QImageProcessingPipeline*>(Base::_opts) ) {
      pp->disconnect(this);
    }

    if( (ThisClass::_opts = dynamic_cast<c_pipeline_type*>(pipeline)) ) {
      QObject::connect(pipeline, &QImageProcessingPipeline::stateChanged,
          this, &ThisClass::onPipelineStateChanged,
          Qt::QueuedConnection);
    }

    ThisClass::updateControls();
  }

  QImageProcessingPipeline* currentPipeline() const final
  {
    return dynamic_cast<QImageProcessingPipeline*>(Base::_opts);
  }

protected:
  void onPipelineStateChanged() final
  {
  }
};


template<class c_pipeline_type>
class QImageProcessingPipelineTemplate :
    public QImageProcessingPipeline,
    public c_pipeline_type
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
    c_pipeline_type(name.toStdString(), input_sequence)
  {
  }

  QString className() const override
  {
    return c_pipeline_type::class_name().c_str();
  }

  void setName(const QString & name) override
  {
    c_pipeline_type::set_name(name.toStdString());
  }

  QString getName() const override
  {
    return c_pipeline_type::cname();
  }

  bool serialize(c_config_setting settings, bool save) override
  {
    return c_pipeline_type::serialize(settings, save);
  }

  bool run(const c_input_sequence::sptr & input_sequence = nullptr) override
  {
    return c_pipeline_type::run(input_sequence);
  }

  bool get_display_image(cv::OutputArray frame, cv::OutputArray mask) override
  {
    return c_pipeline_type::get_display_image(frame, mask);
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
