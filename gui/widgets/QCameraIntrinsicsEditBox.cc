/*
 * QCameraIntrinsicsEditBox.cc
 *
 *  Created on: Aug 2, 2023
 *      Author: amyznikov
 */

#include "QCameraIntrinsicsEditBox.h"
#include <core/settings/camera_settings.h>
#include <core/readdir.h>


namespace {

struct c_named_camera {
  std::string name;
  c_camera_intrinsics intrinsics;
};

static std::vector<c_named_camera> known_cameras;

static const std::string config_path =
    expand_path("~/.config/SerStacker/known_cameras.cfg");


void load_known_cameras()
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

    c_named_camera camera;

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

        known_cameras.emplace_back(camera);
      }
    }
  }
}

void save_known_cameras()
{
  c_config config;

  c_config_setting settings =
      config.root().add_list("cameras");

  const int n =
      known_cameras.size();

  for( int i = 0; i < n; ++i ) {

    c_config_setting item =
        settings.add_group();

    save_settings(item, "name", known_cameras[i].name);
    save_settings(item, "image_size", known_cameras[i].intrinsics.image_size);
    save_settings(item, "camera_matrix", known_cameras[i].intrinsics.camera_matrix);
    save_settings(item, "dist_coeffs", known_cameras[i].intrinsics.dist_coeffs);
  }

  if( !config.write(config_path) ) {
    CF_ERROR("config.write('%s') fails",
        config_path.c_str());
  }

}

class QCameraSelectionDialog:
    public QDialog
{
public:
  typedef QCameraSelectionDialog ThisClass;
  typedef QDialog Base;

  QCameraSelectionDialog(QWidget * parent = nullptr) :
    Base(parent),
    selectedIndex_(-1)
  {
    QVBoxLayout * listBox = new QVBoxLayout();
    listBox->addWidget(filter_ctl = new QLineEdit(this));
    listBox->addWidget(list_ctl = new QListWidget(this));

    QHBoxLayout * buttonHBox = new QHBoxLayout();
    buttonHBox->addWidget(btnOK = new QPushButton("OK"));
    buttonHBox->addWidget(btnCancel = new QPushButton("Cancel"));
    btnOK->setDefault(true);


    QVBoxLayout * mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(listBox);
    mainLayout->addLayout(buttonHBox);

    list_ctl->setViewMode(QListView::ViewMode::ListMode);
    list_ctl->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
    list_ctl->setSelectionMode(QAbstractItemView::SelectionMode::SingleSelection);

    for( const c_named_camera &c : known_cameras ) {
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

  }


  int selectedIndex() const
  {
    return selectedIndex_;
  }

protected:
  QLineEdit * filter_ctl = nullptr;
  QListWidget * list_ctl = nullptr;
  QPushButton * btnOK = nullptr;
  QPushButton * btnCancel = nullptr;

  int selectedIndex_;

};

}




QCameraIntrinsicsEditBox::QCameraIntrinsicsEditBox(QWidget * parent) :
    Base(parent)
{

  imageSize_ctl =
      add_numeric_box<cv::Size>("Frame size",
          "",
          [this](const cv::Size & v) {
            if ( options_ && options_->image_size != v ) {
              options_->image_size = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](cv::Size * v) {
            if ( options_ ) {
              *v = options_->image_size;
              return true;

            }
            return false;
          });


  addRow("Camera Matrix",
      cameraMatrix_ctl = new QMatrixEdit(3, 3, this));

  connect(this, &ThisClass::populatecontrols,
      [this]() {
        if ( options_ ) {
          cameraMatrix_ctl->setMatrix(cv::Mat(options_->camera_matrix));
        }
      });

  connect(cameraMatrix_ctl, &QMatrixEdit::matrixChanged,
      [this]() {
        if ( options_ && !updatingControls() ) {
          if ( cameraMatrix_ctl->getMatrix(&options_->camera_matrix) ) {
            Q_EMIT parameterChanged();
          }
        }
      });


  distCoeffs_ctl =
      add_numeric_box<std::vector<double>> ("Dist. Coeffs:",
          "",
          [this](const std::vector<double> & v) {
            if ( options_ ) {
              options_->dist_coeffs = v;
              Q_EMIT parameterChanged();
            }
          },
          [this](std::vector<double> * v) {
            if ( options_ ) {
              *v = options_->dist_coeffs;
              return true;

            }
            return false;
          });

  options_ctl =
      add_tool_button("Options...",
          [this](bool /*checked*/) {
            if ( options_ ) {
              showOptionsMenu();
            }
          });

  updateControls();
}

void QCameraIntrinsicsEditBox::showOptionsMenu()
{
  if( known_cameras.empty() ) {

    load_known_cameras();

    if( known_cameras.empty() ) {

      c_named_camera camera;
      camera.name = "KITTI";
      camera.intrinsics.image_size = cv::Size(1242, 375);
      camera.intrinsics.camera_matrix = cv::Matx33d(
          7.215377e+02, 0.000000e+00, 6.095593e+02,
          0.000000e+00, 7.215377e+02, 1.728540e+02,
          0.000000e+00, 0.000000e+00, 1.000000e+00);

      known_cameras.emplace_back(camera);
      save_known_cameras();
    }
  }

  QMenu menu;

  if ( options_ ) {

    if( !known_cameras.empty() ) {

      menu.addAction("Saved Cameras..",
          [this]() {

            QCameraSelectionDialog dialog(this);

            if ( dialog.exec() == QDialog::Accepted ) {

              int seletedIndex =
                  dialog.selectedIndex();

              CF_DEBUG("seletedIndex=%d", seletedIndex);

              if ( seletedIndex >= 0 ) {
                *options_ = known_cameras[seletedIndex].intrinsics;
                updateControls();
                Q_EMIT parameterChanged();
              }
            }

          });
    }


    menu.addAction("Save Camera...",
        [this]() {

            while ( 42 ) {

              QString cameraName =
                    QInputDialog::getText(this, "Camera name required",
                        "Enter Camera Name:");

              if ( !cameraName.isEmpty() ) {

                const std::string cname =
                    cameraName.toStdString();

                const auto pos =
                  std::find_if(known_cameras.begin(), known_cameras.end(),
                      [cname](const c_named_camera & c) -> bool {
                        return c.name == cname;
                      });

                if ( pos != known_cameras.end() ) {
                  QMessageBox::warning(this, "ERROR",
                      qsprintf("Camera named '%s' already exits",
                          cname.c_str()));
                  continue;
                }


                c_named_camera camera;
                camera.name = cname;
                camera.intrinsics = *options_;

                known_cameras.emplace_back(camera);
                save_known_cameras();
              }

              break;
            }

        });
  }

  if( !menu.isEmpty() ) {

    menu.exec(options_ctl->mapToGlobal(QPoint(
        options_ctl->width() / 2,
        options_ctl->height() / 2)));
  }


}

