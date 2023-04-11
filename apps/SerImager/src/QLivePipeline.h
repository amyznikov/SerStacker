/*
 * QLivePipeline.h
 *
 *  Created on: Mar 20, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __QLivePipeline_h__
#define __QLivePipeline_h__

#include <QtCore/QtCore>
#include <gui/widgets/UpdateControls.h>
#include <gui/qimageview/QImageEditor.h>
#include <gui/qimageview/QImageViewMtfDisplayFunction.h>
#include <gui/qgraphicsshape/QGraphicsRectShape.h>
#include <gui/qgraphicsshape/QGraphicsLineShape.h>
#include <gui/qgraphicsshape/QGraphicsTargetShape.h>
#include <gui/widgets/QSettingsWidget.h>
#include <core/io/debayer.h>
#include <core/settings/opencv_settings.h>
#include "camera/QImagingCamera.h"

namespace serimager {

///////////////////////////////////////////////////////////////////////////////

class QLiveDisplayMtfFunction :
    public QImageViewMtfDisplayFunction
{
  Q_OBJECT;
public:
  typedef QLiveDisplayMtfFunction ThisClass;
  typedef QImageViewMtfDisplayFunction Base;

  QLiveDisplayMtfFunction(QImageViewer * imageViewer);

  std::mutex & mutex()
  {
    return mutex_;
  }

  bool isBusy() const
  {
    return isBusy_;
  }

  void getInputDataRange(double * minval, double * maxval) const override;
  void getInputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;
  void getOutputHistogramm(cv::OutputArray H, double * hmin, double * hmax) override;

  void createDisplayImage(cv::InputArray currentImage, cv::InputArray currentMask,
      cv::OutputArray displayImage, int ddepth = CV_8U) override;

protected:
  mutable std::mutex mutex_;
  bool isBusy_ = false;
};

///////////////////////////////////////////////////////////////////////////////

class QLiveDisplay :
    public QImageEditor
{
  Q_OBJECT;
public:
  typedef QLiveDisplay ThisClass;
  typedef QImageEditor Base;

  QLiveDisplay(QWidget * parent = nullptr);
  ~QLiveDisplay();

  void showVideoFrame(const cv::Mat & image, COLORID colorid, int bpp);


  const QLiveDisplayMtfFunction * mtfDisplayFunction() const;
  QLiveDisplayMtfFunction * mtfDisplayFunction();

  void setFrameProcessor(const c_image_processor::sptr & processor);

  QGraphicsRectShape * rectShape() const;
  QGraphicsLineShape * lineShape() const;
  QGraphicsTargetShape * targetShape() const;

Q_SIGNALS:
  void pixmapChanged(QPrivateSignal * p = nullptr);

protected Q_SLOTS:
  void onPixmapChanged();

protected:
  void createShapes();

protected:
  void showEvent(QShowEvent *event) override;
  void hideEvent(QHideEvent *event) override;

protected:
  QLiveDisplayMtfFunction mtfDisplayFunction_;

  QPixmap pixmap_;

  QGraphicsRectShape * rectShape_ = nullptr;
  QGraphicsLineShape * lineShape_ = nullptr;
  QGraphicsTargetShape * targetShape_ = nullptr;
};

///////////////////////////////////////////////////////////////////////////////

class QLivePipeline:
    public QObject
{
  Q_OBJECT;
public:
  typedef QLivePipeline ThisClass;
  typedef QObject Base;

  QLivePipeline(const QString & name, QObject * parent = nullptr);

  std::mutex& mutex();

  virtual const QString & getClassName() const = 0;

  void setName(const QString & name);
  const QString & name() const;

  void setRunning(bool v);
  bool isRunning() const;


  virtual void set_canceled(bool v);
  virtual bool canceled();

  virtual bool serialize(c_config_setting settings, bool save);
  virtual bool initialize_pipeline();
  virtual void cleanup_pipeline();

  virtual bool process_frame(const cv::Mat & image, COLORID colorid, int bpp) = 0;
  virtual bool get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp) = 0;

Q_SIGNALS:
  void runningStateChanged(bool isRunning);

protected:
  virtual std::string create_output_path(const std::string & output_directory) const;
  virtual bool convert_image(const cv::Mat & src, COLORID src_colorid, int src_bpp,
      cv::Mat * dst_image, COLORID dst_colorid, int ddepth) const;
  virtual std::string generate_output_file_name(const std::string & output_path,
      const std::string & ufilename,
      const std::string & postfix,
      const std::string & suffix) const;


protected:
  QString name_;
  std::mutex mtx_;
  std::atomic_bool running_ = false;
  std::atomic_bool canceled_ = false;
};

class QLivePipelineSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLivePipelineSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QLivePipelineSettingsWidget(QWidget * parent = nullptr) :
      Base("", parent)
  {
  }

  QLivePipelineSettingsWidget(const QString & prefix, QWidget * parent = nullptr) :
      Base(prefix, parent)
  {
  }

  virtual const QString & pipelineClassName() const = 0;

  virtual void setCurrentPipeline(QLivePipeline * pipeline) = 0;
  virtual QLivePipeline * currentPipeline() const = 0;

protected Q_SLOTS:
  virtual void onLivePipelineStateChanged(bool isRunnging) {}
};


template<class PipelineType>
class QLivePipelineSettings:
    public QLivePipelineSettingsWidget
{
public:
  typedef QLivePipelineSettings ThisClass;
  typedef QLivePipelineSettingsWidget Base;

  QLivePipelineSettings(QWidget * parent = nullptr) :
      Base("", parent)
  {
  }

  QLivePipelineSettings(const QString & prefix, QWidget * parent = nullptr) :
      Base(prefix, parent)
  {
  }

  const QString & pipelineClassName() const override
  {
    return PipelineType::className();
  }

  void setPipeline(PipelineType * pipeline)
  {
    if( pipeline_ ) {
      pipeline_->disconnect(this);
    }

    if( (pipeline_ = pipeline) ) {
      connect(pipeline_, &QLivePipeline::runningStateChanged,
          this, &ThisClass::onLivePipelineStateChanged,
          Qt::QueuedConnection);
    }

    updateControls();
  }

  PipelineType * pipeline() const
  {
    return pipeline_;
  }

  void setCurrentPipeline(QLivePipeline * pipeline) override
  {
    setPipeline(dynamic_cast<PipelineType*>(pipeline));
  }

  QLivePipeline * currentPipeline() const override
  {
    return pipeline_;
  }

protected:
  // placeholder for overrides
  virtual void update_pipeline_controls()
  {
  }

  void onupdatecontrols() override
  {
    if ( !pipeline_ ) {
      setEnabled(false);
    }
    else {
      Q_EMIT Base::populatecontrols();
      setEnabled(!pipeline_->isRunning());
      update_pipeline_controls();
    }
  }

protected:
  PipelineType * pipeline_ =
      nullptr;
};

///////////////////////////////////////////////////////////////////////////////

class QLivePipelineCollection :
    public QList<QLivePipeline*>
{
public:
  typedef QLivePipelineCollection ThisClass;
  typedef QList<QLivePipeline*> Base;

  class PipelineType
  {
  public:
    typedef std::function<QLivePipeline* (const QString & className)> PipelineFactoryFunction;
    typedef std::function<QLivePipelineSettingsWidget* (QWidget * parent)> SettingsFactoryFunction;

    PipelineType(const QString & className, const QString & tooltip,
        const PipelineFactoryFunction & factory,
        const SettingsFactoryFunction & settingsFactory) :
        className_(className),
        tooltip_(tooltip),
        createInstance_(factory),
        createPipelineSettings_(settingsFactory)
    {
    }

    const QString& className() const
    {
      return className_;
    }

    const QString& tooltip() const
    {
      return tooltip_;
    }

    QLivePipeline* createInstance(const QString & name) const
    {
      return createInstance_(name);
    }

    QLivePipelineSettingsWidget* createSettingsWidgget(QWidget * parent) const
    {
      return createPipelineSettings_(parent);
    }

  protected:
    const QString className_;
    const QString tooltip_;
    const PipelineFactoryFunction createInstance_;
    const SettingsFactoryFunction createPipelineSettings_;
  };


  const QList<PipelineType*> & pipelineTypes() const;

  bool addPipelineClassFactory(const QString & className, const QString & tooltip,
      const PipelineType::PipelineFactoryFunction & factory,
      const PipelineType::SettingsFactoryFunction & settingsFactory);

  const PipelineType * findPipelineClassFactory(const QString & className) const;


  QLivePipeline* addPipeline(const QString & type, const QString & name);
  bool removePipeline(QLivePipeline * pipeline);

  QLivePipeline * findPipeline(const QString & name) const;

  void load(const std::string & cfgfilename = "");
  void save(const std::string & cfgfilename = "") const;

protected:
  QList<PipelineType*>  pipelineTypes_;
  mutable std::string config_filename_;
  static std::string default_config_filename_;
};

///////////////////////////////////////////////////////////////////////////////

class QLivePipelineThread :
    public QThread
{
  Q_OBJECT;
public:
  typedef QLivePipelineThread ThisClass;
  typedef QThread Base;

  QLivePipelineThread(QObject * parent = nullptr);
  ~QLivePipelineThread();

  void finish(bool wait = true);

  void setCamera(const QImagingCamera::sptr & camera);
  const QImagingCamera::sptr& camera() const;

  void setDisplay(QLiveDisplay * display);
  QLiveDisplay* display() const;

  bool startPipeline(QLivePipeline *pipeline);
  QLivePipeline* currentPipeline() const;

  void setDebayer(DEBAYER_ALGORITHM algo);
  DEBAYER_ALGORITHM debayer() const;

protected Q_SLOTS:
  void onRestartAfterException();
  void onCameraStateChanged(QImagingCamera::State oldState,
      QImagingCamera::State newState);

Q_SIGNALS:
  void restartAfterException(QPrivateSignal * );

protected:
  void run() override;

protected:
  QImagingCamera::sptr camera_;
  QLiveDisplay *display_ = nullptr;
  QLivePipeline * pipeline_ = nullptr;

  std::atomic<DEBAYER_ALGORITHM> debayer_ = DEBAYER_NN;
  std::atomic_bool finish_ = false;
};

///////////////////////////////////////////////////////////////////////////////

class QLivePipelineSelectionWidget :
    public QFrame,
    public HasUpdateControls
{
  Q_OBJECT;
public:
  typedef QLivePipelineSelectionWidget ThisClass;
  typedef QFrame Base;

  QLivePipelineSelectionWidget(QWidget * parent = nullptr);

  void setPipelineCollection(QLivePipelineCollection * pipelines);
  QLivePipelineCollection * pipelines() const;

  void setLiveThread(QLivePipelineThread * liveThread);
  QLivePipelineThread * liveThread() const;

  QLivePipeline * selectedPipeline() const;

Q_SIGNALS:
  void parameterChanged();

protected Q_SLOTS:
  void onPipelinesComboboxCurrentIndexChanged(int);
  void onStartStopCtlClicked();
  void onMenuCtlClicked();
  void onAddLivePipelineClicked();
  void onRemoveLivePipelineClicked();
  void onRenameLivePipelineClicked();
  void onLiveThreadStateChanged();

protected:
  void onupdatecontrols() override;
  void populatepipelines();

protected:
  QLivePipelineCollection * pipelineCollection_ = nullptr;
  QLivePipelineThread * liveThread_ = nullptr;

  QVBoxLayout * layout_ = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QComboBox * combobox_ctl = nullptr;
  QToolButton * startStop_ctl = nullptr;
  QToolButton * menuButton_ctl = nullptr;

  QScrollArea * scrollArea_ctl = nullptr;
  QList<QLivePipelineSettingsWidget *> settingsWidgets_;
};

///////////////////////////////////////////////////////////////////////////////


class QLiveThreadSettingsWidget :
    public QSettingsWidget
{
  Q_OBJECT;
public:
  typedef QLiveThreadSettingsWidget ThisClass;
  typedef QSettingsWidget Base;

  QLiveThreadSettingsWidget(QWidget * parent = nullptr);
  QLiveThreadSettingsWidget(QLivePipelineThread * liveThread, QWidget * parent = nullptr);

  void setLiveThread(QLivePipelineThread * liveThread);
  QLivePipelineThread * liveThread() const;

protected:
  void onupdatecontrols() override;

protected:
  QLivePipelineThread * liveThread_ = nullptr;
  QEnumComboBox<DEBAYER_ALGORITHM> * debayer_ctl = nullptr;
};

class QLiveThreadSettingsDialogBox :
    public QDialog
{
  Q_OBJECT;
public:
  typedef QLiveThreadSettingsDialogBox ThisClass;
  typedef QDialog Base;

  QLiveThreadSettingsDialogBox(QWidget * parent = nullptr);

  void setLiveThread(QLivePipelineThread * liveThread);
  QLivePipelineThread * liveThread() const;

Q_SIGNALS:
  void visibilityChanged(bool visible);

protected:
  void closeEvent(QCloseEvent *) override;
  void showEvent(QShowEvent *e) override;
  void hideEvent(QHideEvent *e) override;

protected:
  QVBoxLayout * layout_ = nullptr;
  QLiveThreadSettingsWidget * setiingsWidget_ = nullptr;
};

///////////////////////////////////////////////////////////////////////////////

} /* namespace serimager */

#endif /* __QLivePipeline_h__ */
