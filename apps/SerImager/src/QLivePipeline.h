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

  virtual bool processFrame(const cv::Mat & image, COLORID colorid, int bpp) = 0;
  virtual bool getDisplayImage(cv::Mat * displayImage, COLORID * colorid, int *bpp) = 0;

  virtual bool convertImage(const cv::Mat & src, COLORID src_colorid, int src_bpp,
      cv::Mat * dst_image, COLORID dst_colorid, int ddepth) const;

protected:
  QString name_;
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

  void load();
  void save();

protected:
  QList<PipelineType*>  pipelineTypes_;
};


class QLivePipelineSettingsWidget :
    public QSettingsWidget
{
};


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

protected:
  void onupdatecontrols() override;
  void populatepipelines();

protected:
  QLivePipelineCollection * pipelineCollection_ = nullptr;

  QVBoxLayout * layout_ = nullptr;
  QToolBar * toolbar_ctl = nullptr;
  QComboBox * combobox_ctl = nullptr;
  QToolButton * startStop_ctl = nullptr;
  QToolButton * menuButton_ctl = nullptr;
};


} /* namespace serimager */


//Q_DECLARE_METATYPE(serimager::QLivePipeline*);

#endif /* __QLivePipeline_h__ */