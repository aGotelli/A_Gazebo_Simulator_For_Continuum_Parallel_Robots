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



#include "parameters_vector_saving.h"

ParametersVectorSaving::ParametersVectorSaving(const QString &file_name_,
                                               const std::vector<float> &parameters_vector_,
                                               QWidget *parent_widget)
    : QWidget(parent_widget),
      file_name(file_name_),
      parameters_vector(parameters_vector_)
{
    this->setObjectName(QString::fromUtf8("ParametersVectorNaming"));
    this->resize(367, 154);
    this->setWindowTitle(QApplication::translate("ParametersVectorNaming", "Save Parameters Vector", nullptr));

    gridLayout = new QGridLayout(this);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));

    parameters_vector_gridLayout = new QGridLayout();
    parameters_vector_gridLayout->setObjectName(QString::fromUtf8("parameters_vector_gridLayout"));
    parameters_vector_name_label = new QLabel(this);
    parameters_vector_name_label->setObjectName(QString::fromUtf8("parameters_vector_name_label"));
    parameters_vector_name_label->setMaximumSize(QSize(175, 25));
    parameters_vector_name_label->setText(QApplication::translate("ParametersVectorNaming", "Parameters Vector Name", nullptr));

    parameters_vector_gridLayout->addWidget(parameters_vector_name_label, 0, 0, 1, 1);

    parameters_vector_name_lineEdit = new QLineEdit(this);
    parameters_vector_name_lineEdit->setObjectName(QString::fromUtf8("parameters_vector_name_lineEdit"));
    parameters_vector_name_lineEdit->setMaximumSize(QSize(150, 25));

    parameters_vector_gridLayout->addWidget(parameters_vector_name_lineEdit, 0, 1, 1, 1);


    gridLayout->addLayout(parameters_vector_gridLayout, 0, 0, 1, 1);

    brief_gridLayout = new QGridLayout();
    brief_gridLayout->setObjectName(QString::fromUtf8("brief_gridLayout"));
    brief_label = new QLabel(this);
    brief_label->setObjectName(QString::fromUtf8("brief_label"));
    brief_label->setMaximumSize(QSize(35, 25));
    brief_label->setText(QApplication::translate("ParametersVectorNaming", "Brief", nullptr));

    brief_gridLayout->addWidget(brief_label, 0, 0, 1, 1);

    brief_textEdit = new QTextEdit(this);
    brief_textEdit->setObjectName(QString::fromUtf8("brief_textEdit"));
    brief_textEdit->setMaximumSize(QSize(260, 70));

    brief_gridLayout->addWidget(brief_textEdit, 0, 1, 2, 1);

    brief_verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

    brief_gridLayout->addItem(brief_verticalSpacer, 1, 0, 1, 1);

    gridLayout->addLayout(brief_gridLayout, 1, 0, 1, 1);

    buttonBox = new QDialogButtonBox(this);
    buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
    QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(buttonBox->sizePolicy().hasHeightForWidth());
    buttonBox->setSizePolicy(sizePolicy);
    buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

    gridLayout->addWidget(buttonBox, 2, 0, 1, 1);



    QMetaObject::connectSlotsByName(this);


    QObject::connect(buttonBox->buttons()[1], &QPushButton::clicked, [this](){
        this->close();
    });

    QObject::connect(buttonBox->buttons()[0], &QPushButton::clicked, [this](){
        this->SaveParametersVector(this->parameters_vector_name_lineEdit->text(), brief_textEdit->toPlainText());

        emit this->Saved();

        this->close();
    });
}

void ParametersVectorSaving::SaveParametersVector(const QString &section_name, const QString &brief)
{
if (file_name.isEmpty())
      return;




  QFile robot_configurations_file(file_name);

  if (!robot_configurations_file.open(QIODevice::ReadWrite)) {
      QMessageBox::information(this, tr("Unable to open file"),
          robot_configurations_file.errorString());
      return;
  }



  YAML::Node original_file = YAML::LoadFile( file_name.toStdString() );
  YAML::Node launch_settings  = original_file["launching"];
  YAML::Node original_parameters_vectors = original_file["user_parameters_vectors"];


  YAML::Emitter emitter;

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "launching";
  emitter << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "package_name";
      emitter << launch_settings["package_name"];
      emitter << YAML::Key << "launch_files";
      emitter << YAML::Value << YAML::Flow << launch_settings["launch_files"];
  emitter << YAML::EndMap;
  emitter << YAML::Key << "number_of_limbs";
  emitter << original_file["number_of_limbs"];
  emitter << YAML::Key << "initial_guess";
  emitter << original_file["initial_guess"];
  emitter << YAML::Newline;
  emitter << YAML::Key << "rod";
  emitter << original_file["rod"];
  emitter << YAML::Newline;
  emitter << YAML::Key << "user_parameters_vectors";
  emitter << YAML::Value << YAML::BeginMap;
      for(YAML::const_iterator it=original_parameters_vectors.begin();it != original_parameters_vectors.end();++it) {
         std::string section_name = it->first.as<std::string>();
         YAML::Node parameters_vector_data = it->second;

         std::vector<float> parameters = parameters_vector_data["parameters"].as<std::vector<float>>();
         //QList<QVariant> parameters_vector_ = parameters_vector_data["parameters"].as<QList<QVariant>>();
         std::string brief = parameters_vector_data["brief"].as<std::string>();

         AddSection(emitter, section_name, parameters, brief);
      }
      AddSection(emitter, section_name.toStdString(), parameters_vector, brief.toStdString());
  emitter << YAML::EndMap;
  emitter << YAML::EndMap;



  std::ofstream fout( file_name.toStdString() );
  fout << emitter.c_str();
}

void ParametersVectorSaving::AddSection(YAML::Emitter &emitter, const std::string &section_name,
                                        const std::vector<float> &parameters, const std::string &brief)
{

    emitter << YAML::Key << section_name;
    emitter << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "parameters" << YAML::Value << YAML::Flow << parameters;
        emitter << YAML::Key << "brief" << YAML::Value << brief;
    emitter << YAML::EndMap;
    emitter << YAML::Newline;

}
