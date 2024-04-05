/*
 * QCameraMatrixSelectionDialogBox.cc
 *
 *  Created on: Oct 3, 2023
 *      Author: amyznikov
 */

#include "QCameraMatrixDialogBox.h"

#include <gui/widgets/qsprintf.h>
#include <core/ssprintf.h>
#include <core/readdir.h>
#include <core/debug.h>

static const std::string config_path =
    expand_path("~/.config/SerStacker/saved_cameras.cfg");



QCameraMatrixDialogBox::QCameraMatrixDialogBox(QWidget * parent) :
  Base(parent)
{

  QVBoxLayout * mainLayout = new QVBoxLayout(this);

//  QHBoxLayout * l1 = new QHBoxLayout();
//  QHBoxLayout * l2 = new QHBoxLayout();
//  QVBoxLayout * l3 = new QVBoxLayout();


  // Bottom button box layout
//  l2->addWidget(btnOK = new QPushButton("Select"));
//  l2->addWidget(btnCancel = new QPushButton("Close"));
//  btnOK->setDefault(true);
//
//
//  l1->addLayout(l3);
//  mainLayout->addLayout(l1, 100);
//  mainLayout->addLayout(l2, 1);

  ////////////

//  QVBoxLayout * listBox =
//      new QVBoxLayout();
//
//  listBox->addWidget(filter_ctl = new QLineEdit(this));
//  listBox->addWidget(list_ctl = new QListWidget(this));
//
//  QHBoxLayout * buttonHBox = new QHBoxLayout();
//  buttonHBox->addWidget(btnOK = new QPushButton("Select"));
//  buttonHBox->addWidget(btnCancel = new QPushButton("Close"));
//  btnOK->setDefault(true);
//
//
//  QVBoxLayout * mainLayout = new QVBoxLayout(this);
//  mainLayout->addLayout(listBox);
//  mainLayout->addLayout(buttonHBox);
//
//  list_ctl->setViewMode(QListView::ViewMode::ListMode);
//  list_ctl->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
//  list_ctl->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);
//  list_ctl->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);

  /////////////


  load_cameras();

  for( const Camera &c : cameras_ ) {
    list_ctl->addItem(c.name.c_str());
  }

  connect(filter_ctl, &QLineEdit::textChanged,
      [this](const QString & text) {

        if ( text.isEmpty() ) {
          for ( int i = 0, n = list_ctl->count(); i < n; ++i ) {
            list_ctl->item(i)->setHidden(false);
          }
        }
        else {

          for ( int i = 0, n = list_ctl->count(); i < n; ++i ) {

            QListWidgetItem * item =
                list_ctl->item(i);

            if ( item->text().contains(text, Qt::CaseInsensitive) ) {
              item->setHidden(false);
            }
            else {
              item->setHidden(true);
            }
          }
        }
      });

  connect(btnCancel, &QPushButton::clicked,
      this, &ThisClass::reject);

  connect(btnOK, &QPushButton::clicked,
      [this]() {
        selectedIndex_ = list_ctl->currentRow();
        if ( selectedIndex_ >= 0 && !(list_ctl->item(selectedIndex_)->isHidden())) {
          accept();
        }
      });

  connect(list_ctl, &QListWidget::itemDoubleClicked,
      [this](QListWidgetItem *item) {
        if ( list_ctl->currentItem() != item ) {
          list_ctl->setCurrentItem(item);
        }
        selectedIndex_ = list_ctl->currentRow();
        if ( selectedIndex_ >= 0 && !(list_ctl->item(selectedIndex_)->isHidden())) {
          accept();
        }
      });

  connect(list_ctl, &QListWidget::customContextMenuRequested,
      [this](const QPoint & pos) {

        const int selectedIndex = list_ctl->indexAt(pos).row();
        if ( selectedIndex < 0 ) {
          return;
        }

        QListWidgetItem * item = list_ctl->item(selectedIndex);
        if ( !item ) {
          return;
        }


        QMenu menu;

        const std::string cname =
            item->text().toStdString();

        menu.addAction(qsprintf("Delete %s", cname.c_str()),
              [this, selectedIndex]() {

                const int resp = QMessageBox::question(this, "Confirmation required",
                    qsprintf("Delete camera parameters for '%s' ?", cameras_[selectedIndex].name.c_str()),
                    QMessageBox::StandardButtons(QMessageBox::Yes | QMessageBox::No));

                if ( resp != QMessageBox::Yes ) {
                  return;
                }

                cameras_.erase(cameras_.begin() + selectedIndex);
                save_cameras();

                delete list_ctl->takeItem(selectedIndex);
              });

          menu.exec(list_ctl->mapToGlobal(pos));

      });

}



void QCameraMatrixDialogBox::load_cameras()
{
  if( !file_readable(config_path) ) {
    return;
  }

  c_config config;

  if( !config.read(config_path) ) {
    CF_ERROR("config.read('%s') fails", config_path.c_str());
    return;
  }

  c_config_setting settings =
      config.root()["cameras"];


  if ( settings.isList() || settings.isArray()  ) {

    Camera camera;

    const int n =
        settings.length();

    for ( int i = 0; i < n; ++i ) {

      c_config_setting item =
          settings.get_element(i);

      if ( item.isGroup() ) {

        camera.name.clear();
        camera.intrinsics.dist_coeffs.clear();

        if( !load_settings(item, "name", &camera.name) || camera.name.empty() ) {
          CF_ERROR("name not set for camera element %d", i);
          continue;
        }

        if( !load_settings(item, "image_size", &camera.intrinsics.image_size) ) {
          CF_ERROR("image_size not set for camera element %d", i);
          continue;
        }

        if( !load_settings(item, "camera_matrix", &camera.intrinsics.camera_matrix) ) {
          CF_ERROR("camera_matrix not set for camera element %d", i);
          continue;
        }

        load_settings(item, "dist_coeffs", &camera.intrinsics.dist_coeffs);

        cameras_.emplace_back(camera);
      }
    }
  }
}

void QCameraMatrixDialogBox::save_cameras()
{
  c_config config;

  c_config_setting settings =
      config.root().add_list("cameras");

  const int n =
      cameras_.size();

  for( int i = 0; i < n; ++i ) {

    c_config_setting item =
        settings.add_group();

    const Camera & camera =
        cameras_[i];

    save_settings(item, "name", camera.name);
    save_settings(item, "image_size", camera.intrinsics.image_size);
    save_settings(item, "camera_matrix", camera.intrinsics.camera_matrix);
    save_settings(item, "dist_coeffs", camera.intrinsics.dist_coeffs);
  }

  if( !config.write(config_path) ) {
    CF_ERROR("config.write('%s') fails",
        config_path.c_str());
  }

}
