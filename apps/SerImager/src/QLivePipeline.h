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
#include <core/settings/opencv_settings.h>
#include "camera/QImagingCamera.h"
#include "QVideoFrameDisplay.h"

namespace serimager {

class QLivePipeline:
    public QObject
{
  Q_OBJECT;
public:
  typedef QLivePipeline ThisClass;
  typedef QObject Base;

  QLivePipeline(const QString & name, QObject * parent = nullptr);

  void setName(const QString & name);
  const QString & name() const;

  void set_running(bool v);
  bool is_running() const;

  virtual void set_canceled(bool v);
  virtual bool canceled();

  virtual bool serialize(c_config_setting settings, bool save);
  virtual bool initialize_pipeline();
  virtual void cleanup_pipeline();

  virtual bool process_frame(const cv::Mat & image, COLORID colorid, int bpp) = 0;
  virtual bool get_display_image(cv::Mat * displayImage, COLORID * colorid, int *bpp) = 0;
  virtual bool convert_image(const cv::Mat & src, COLORID src_colorid, int src_bpp,
      cv::Mat * dst_image, COLORID dst_colorid, int ddepth) const;
  virtual std::string create_output_path(const std::string & output_directory) const;

  virtual std::string generate_output_file_name(const std::string & output_path,
      const std::string & ufilename,
      const std::string & postfix,
      const std::string & suffix) const;

Q_SIGNALS:
  void state_changed(bool is_running);

protected:
  QString name_;
  std::atomic_bool running_ = false;
  std::atomic_bool canceled_ = false;
};


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

  void setDisplay(QVideoFrameDisplay * display);
  QVideoFrameDisplay* display() const;

  bool startPipeline(QLivePipeline *pipeline);
  QLivePipeline* currentPipeline() const;

  void setDebayer(DEBAYER_ALGORITHM algo);
  DEBAYER_ALGORITHM debayer() const;

Q_SIGNALS:
  void pipelineStarted(QLivePipeline *pipeline);
  void pipelineFinished(QLivePipeline *pipeline);

protected Q_SLOTS:
  void onCameraStateChanged(QImagingCamera::State oldState,
      QImagingCamera::State newState);

protected:
  void run() override;

protected:
  QImagingCamera::sptr camera_;
  QVideoFrameDisplay *display_ = nullptr;
  QLivePipeline * pipeline_ = nullptr;

  std::atomic<DEBAYER_ALGORITHM> debayer_ = DEBAYER_NN;
  std::atomic_bool finish_ = false;
};



class QLivePipelineCollection :
    public QList<QLivePipeline*>
{
public:
  typedef QLivePipelineCollection ThisClass;
  typedef QList<QLivePipeline*> Base;

  class PipelineType
  {
  public:
    typedef std::function<QLivePipeline* (const QString & name)> factoryFunction;

    PipelineType(const QString & name, const QString & tooltip, const factoryFunction & factory) :
        name_(name),
        tooltip_(tooltip),
        createInstance_(factory)
    {
    }

    const QString& name() const
    {
      return name_;
    }

    const QString& tooltip() const
    {
      return tooltip_;
    }

    QLivePipeline* createInstance(const QString & name) const
    {
      return createInstance_(name);
    }

  protected:
    const QString name_;
    const QString tooltip_;
    const factoryFunction createInstance_;
  };


  const QList<PipelineType*> & pipelineTypes() const;

  bool addPipelineType(const QString & name, const QString & tooltip,
      const PipelineType::factoryFunction & factory);

  QLivePipeline * addPipeline(const QString & type, const QString & name);

  QLivePipeline * findPipeline(const QString & name) const;

  void load(const std::string & cfgfilename = "");
  void save(const std::string & cfgfilename = "") const;

protected:
  QList<PipelineType*>  pipelineTypes_;
  mutable std::string config_filename_;
  static std::string default_config_filename_;
};


class QLiveStereoCalibrationOptions;
class QLiveCameraCalibrationOptions;
class QLiveRegularStereoOptions;

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
  QLiveStereoCalibrationOptions * stereoCalibrationOptions_ctl = nullptr;
  QLiveCameraCalibrationOptions * cameraCalibrationOptions_ctl = nullptr;
  QLiveRegularStereoOptions * regularStereoOptions_ctl = nullptr;

};



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


} /* namespace serimager */


//Q_DECLARE_METATYPE(serimager::QLivePipeline*);

#endif /* __QLivePipeline_h__ */
