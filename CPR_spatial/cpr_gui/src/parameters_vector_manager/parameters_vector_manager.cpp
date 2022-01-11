/*!
MIT License

Copyright (c) 2022 Gotelli Andrea

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/




#include "parameters_vector_manager.h"

ParametersVectorManager::ParametersVectorManager(QWidget *parent_widget) :
    QWidget(parent_widget)
{
    setObjectName(QString::fromUtf8("ParametersVectorManager"));
    resize(400, 300);
    setWindowTitle(QApplication::translate("ParametersVectorManager", "ParametersVectorManager", nullptr));

    verticalLayout = new QVBoxLayout(this);

    listWidget = new QListWidget(this);
    listWidget->setObjectName(QString::fromUtf8("listWidget"));
    listWidget->setGeometry(QRect(70, 30, 256, 192));
    verticalLayout->addWidget(listWidget);

    buttonBox = new QDialogButtonBox(this);
    buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
    buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
    verticalLayout->addWidget(buttonBox);


    QString path = QString("/home/andrea/QtCreator/latest/catkin/src/A-Gazebo-Simulator-for-Continuum-Parallel-Robots/robot_physics/launch");

    fileName = QFileDialog::getOpenFileName(this, tr("Open A Robot Configuration File"), path, tr("YAML File (*.yaml)"));


    if (fileName.isEmpty())
        return;

    QFile robot_configurations_file(fileName);

    if (!robot_configurations_file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(this, tr("Unable to open file"),
            robot_configurations_file.errorString());
        return;
    }

    YAML::Node node = YAML::LoadFile( fileName.toStdString() );

    YAML::Node parameters_vectors = node["user_parameters_vectors"];

    for(YAML::const_iterator it=parameters_vectors.begin();it != parameters_vectors.end();++it) {
       std::string key = it->first.as<std::string>();
       YAML::Node parameters_vector_data = it->second;

       std::vector<double> parameters_vector = parameters_vector_data["parameters"].as<std::vector<double>>();
       //QList<QVariant> parameters_vector_ = parameters_vector_data["parameters"].as<QList<QVariant>>();
       std::string brief = parameters_vector_data["brief"].as<std::string>();

       QListWidgetItem *item = new QListWidgetItem();
       item->setText( QString::fromStdString(key) );
       item->setToolTip( QString::fromStdString(brief) );

       QList<QVariant> list_;
       list_.reserve(parameters_vector.size());
       std::copy(parameters_vector.begin(), parameters_vector.end(), std::back_inserter(list_));

       //QList<double> list_of_parameters = QList<double>::fromVector( QVector<double>::fromStdVector(parameters_vector));
       //item->setData(0, list_of_parameters);
       QVariant data(list_);

       item->setData(Qt::UserRole, data);
       listWidget->addItem(item);
    }


//    QObject::connect(buttonBox->buttons()[0], &QPushButton::clicked, [this](){
//      auto current_item = this->listWidget->currentItem();

//      if(current_item != nullptr) {

//        QList<QVariant> item_data = current_item->data(Qt::UserRole).toList();

//        emit this->ParametersVectorChosen( item_data.toVector().toStdVector() );
//      }

//    });

//    QObject::connect(buttonBox->buttons()[1], &QPushButton::clicked, [this](){
//      this->close();
//    });

//    QObject::connect(&naming_window, &ParametersVectorNaming::SavingConfig, this, &ParametersVectorManager::SaveParametersVector);

    QMetaObject::connectSlotsByName(this);
}


