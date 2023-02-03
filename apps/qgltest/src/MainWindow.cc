/*
 * MainWindow.cc
 *
 *  Created on: Dec 21, 2022
 *      Author: amyznikov
 */

#include "MainWindow.h"
#include <gui/widgets/style.h>
#include <gui/widgets/qsprintf.h>

namespace qgltest {

#define ICON_histogram        ":/qgltest/icons/histogram.png"
#define ICON_display          ":/qgltest/icons/display.png"
#define ICON_shapes           ":/qgltest/icons/shapes.png"
#define ICON_roi              ":/qgltest/icons/roi.png"
#define ICON_line             ":/qgltest/icons/line.png"
#define ICON_target           ":/qgltest/icons/target.png"

namespace {

template<class Obj, typename Fn>
QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip,
    Obj * receiver, Fn fn)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);
  action->setCheckable(true);

  QObject::connect(action, &QAction::triggered, receiver, fn);

  return action;
}

template<typename Slot>
QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip, Slot && slot)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);
  action->setCheckable(true);

  QObject::connect(action, &QAction::triggered, slot);

  return action;
}

QAction* createCheckableAction(const QIcon & icon, const QString & text, const QString & tooltip)
{
  QAction *action = new QAction(icon, text);
  action->setToolTip(tooltip);
  action->setCheckable(true);

  return action;
}

QToolButton* createToolButtonWithPopupMenu(QAction * defaultAction, QMenu * menu)
{
  QToolButton *tb = new QToolButton();
  tb->setToolButtonStyle(Qt::ToolButtonIconOnly);
  tb->setIcon(defaultAction->icon());
  tb->setText(defaultAction->text());
  tb->setToolTip(defaultAction->toolTip());
  tb->setCheckable(defaultAction->isCheckable());
  tb->setPopupMode(QToolButton::ToolButtonPopupMode::MenuButtonPopup);
  tb->setDefaultAction(defaultAction);
  tb->setMenu(menu);
  return tb;
}

QWidget* addStretch(QToolBar * toolbar)
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  toolbar->addWidget(stretch);
  return stretch;
}

QWidget* createStretch()
{
  QWidget *stretch = new QWidget();
  stretch->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  return stretch;
}

} // namespace

MainWindow::MainWindow(QWidget * parent) :
    Base(parent)
{
  setWindowIcon(getIcon(":/qgltest/icons/app-icon.png"));

  setCentralWidget(glView_ = new QTestGLView(this));

  setDockOptions(AnimatedDocks | AllowTabbedDocks | AllowNestedDocks | GroupedDragging);
  setCorner(Qt::TopLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::TopRightCorner, Qt::RightDockWidgetArea);
  setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
  setCorner(Qt::BottomRightCorner, Qt::BottomDockWidgetArea);

  setupStatusbar();
  setupMainMenu();
  setupMainToolbar();
  setupGLViewSettingsDock();

  restoreState();

  QApplication::instance()->installEventFilter(this);
}

MainWindow::~MainWindow()
{
  saveState();
}

bool MainWindow::eventFilter(QObject * watched, QEvent * event)
{
  if( event->type() == QEvent::Wheel ) {

    if( const auto *combo = dynamic_cast<const QComboBox*>(watched) ) {
      if( !combo->hasFocus() ) {
        return true;
      }
    }
    //  spin_ctl->setFocusPolicy(Qt::FocusPolicy::StrongFocus) not works
    else if( const auto *spin = dynamic_cast<const QSpinBox*>(watched) ) {
      if( !spin->hasFocus() ) {
        return true;
      }
    }
    else if( const auto *slider = dynamic_cast<const QSlider*>(watched) ) {
      if( !slider->hasFocus() ) {
        return true;
      }
    }
  }

  return Base::eventFilter(watched, event);
}

void MainWindow::saveState()
{
  QSettings settings;
  settings.setValue("MainWindow/Geometry", Base::saveGeometry());
  settings.setValue("MainWindow/State", Base::saveState());
}

void MainWindow::restoreState()
{
  QSettings settings;
  Base::restoreGeometry(settings.value("MainWindow/Geometry").toByteArray());
  Base::restoreState(settings.value("MainWindow/State").toByteArray());
}

void MainWindow::setupMainMenu()
{
  menuBar()->setNativeMenuBar(false);

  ///////////////////////////////////////////////////////////////////

  menuFile_ =
      menuBar()->addMenu("&File");

  menuFile_->addSeparator();

  menuFile_->addAction("Quit", this,
      &ThisClass::close);

  ///////////////////////////////////////////////////////////////////

  menuView_ =
      menuBar()->addMenu("&View");

  /////////////////////////////////////
}


void MainWindow::setupMainToolbar()
{
//  menuBar()->setCornerWidget(manToolbar_ =
//      new QToolBar(this),
//      Qt::TopRightCorner);
//
//  manToolbar_->setContentsMargins(0, 0, 0, 0);
//  manToolbar_->setToolButtonStyle(Qt::ToolButtonIconOnly);
//  manToolbar_->setIconSize(QSize(32, 18));
//
//  ///////////////////////////////////////////////////////////////////
//
//  manToolbar_->addWidget(rectShapeActionsButton_ =
//      createToolButtonWithPopupMenu(showRectShapeAction_,
//          &rectShapeActionsMenu_));
//
//  manToolbar_->addWidget(lineShapeActionsButton_ =
//      createToolButtonWithPopupMenu(showLineShapeAction_,
//          &lineShapeActionsMenu_));
//
//  manToolbar_->addWidget(targetShapeActionsButton_ =
//      createToolButtonWithPopupMenu(showTargetShapeAction_,
//          &targetShapeActionsMenu_));
//
//
//
//
//  ///////////////////////////////////////////////////////////////////
//
//
//  manToolbar_->addAction(showFrameProcessorAction_);
//  manToolbar_->addAction(showMtfControlAction_);
//
  ///////////////////////////////////////////////////////////////////

}

void MainWindow::setupStatusbar()
{
  QStatusBar *sb = statusBar();

//  sb->addWidget(exposure_status_ctl = new QLabel(this));
//  sb->addWidget(mousepos_ctl = new QLabel(this));
//  sb->addPermanentWidget(capture_status_ctl = new QLabel("", this));
}

void MainWindow::setupGLViewSettingsDock()
{
  glViewSettingsDock_ =
      addCustomDock(this,
          Qt::LeftDockWidgetArea,
          "glViewSettingsDock_",
          "GLViewSettings",
          glViewSettings_ = new QTestGLViewSettings(this),
          menuView_);

  glViewSettings_->setGLView(glView_);
}


} /* namespace qgltest */
