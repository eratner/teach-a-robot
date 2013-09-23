#ifndef HELPERS_H
#define HELPERS_H

#include <ros/ros.h>

#include <dviz_core/goal.h>
#include <dviz_core/object.h>

#include <QDialog>
#include <QComboBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>

namespace demonstration_visualizer
{

/**
 * @brief A struct for maintaining client-side information about the current 
 *        user demonstration.
 */
struct UserDemonstrationInfo
{
  UserDemonstrationInfo()
  : start_time_(), end_time_(), last_goal_time_(), started_(false), goals_completed_(0)
  {

  }

  void reset()
  {
    start_time_ = ros::Time();
    end_time_ = ros::Time();
    last_goal_time_ = ros::Time();
  }

  void start()
  {
    start_time_ = ros::Time::now();
    last_goal_time_ = ros::Time::now();
    started_ = true;
  }
  
  ros::Duration stop()
  {
    if(started_)
    {
      started_ = false;
      end_time_ = ros::Time::now();
      return (end_time_ - start_time_);
    }

    return ros::Duration(0.0);
  }

  ros::Duration goalComplete()
  {
    goals_completed_++;

    ros::Duration time_to_complete = ros::Time::now() - last_goal_time_;

    last_goal_time_ = ros::Time::now();

    return time_to_complete;
  }

  ros::Time start_time_;
  ros::Time end_time_;
  ros::Time last_goal_time_;
  bool started_;
  int goals_completed_;

};

class AddGoalDialog : public QDialog
{
Q_OBJECT
public:
  AddGoalDialog()
    : goal_type_(Goal::PICK_UP), object_id_(0), ignore_yaw_(false)
  {
    QVBoxLayout *layout = new QVBoxLayout();

    // Select the goal type.
    QHBoxLayout *select_goal_layout = new QHBoxLayout();
    QLabel *select_goal_label = new QLabel("Goal Type: ");
    select_goal_type_ = new QComboBox();
    select_goal_type_->setInsertPolicy(QComboBox::InsertAtBottom);
    for(int i = 0; i < Goal::NumGoalTypes; ++i)
    {
      select_goal_type_->addItem(QString(Goal::GoalTypeNames[i]));
    }
    select_goal_layout->addWidget(select_goal_label);
    select_goal_layout->addWidget(select_goal_type_);
    layout->addLayout(select_goal_layout);

    // Select the object (if goal type is PICK_UP).
    QHBoxLayout *select_object_layout = new QHBoxLayout();
    QLabel *select_object_label = new QLabel("Object: ");
    select_object_ = new QComboBox();
    select_object_->setInsertPolicy(QComboBox::InsertAtBottom);
    select_object_layout->addWidget(select_object_label);
    select_object_layout->addWidget(select_object_);
    layout->addLayout(select_object_layout);

    QHBoxLayout *ignore_yaw_layout = new QHBoxLayout();
    QLabel *ignore_yaw_label = new QLabel("Ignore yaw? ");
    ignore_yaw_layout->addWidget(ignore_yaw_label);
    QCheckBox *ignore_yaw_checkbox = new QCheckBox();
    ignore_yaw_layout->addWidget(ignore_yaw_checkbox);
    layout->addLayout(ignore_yaw_layout);

    QHBoxLayout *description_layout = new QHBoxLayout();
    QLabel *description_label = new QLabel("Description: ");
    QLineEdit *description_edit = new QLineEdit();
    description_layout->addWidget(description_label);
    description_layout->addWidget(description_edit);
    layout->addLayout(description_layout);

    QHBoxLayout *confirm_layout = new QHBoxLayout();
    QPushButton *ok_button = new QPushButton("Ok");
    QPushButton *cancel_button = new QPushButton("Cancel");
    confirm_layout->addWidget(ok_button);
    confirm_layout->addWidget(cancel_button);
    layout->addLayout(confirm_layout);

    connect(ok_button, SIGNAL(clicked()), this, SLOT(accept()));
    connect(cancel_button, SIGNAL(clicked()), this, SLOT(reject()));

    connect(select_goal_type_, SIGNAL(currentIndexChanged(int)), this, SLOT(goalTypeChanged(int)));
    connect(select_object_, SIGNAL(currentIndexChanged(int)), this, SLOT(objectChanged(int)));
    connect(description_edit, SIGNAL(textChanged(const QString &)), 
	    this, SLOT(setDescription(const QString &)));
    connect(ignore_yaw_checkbox, SIGNAL(stateChanged(int)), this, SLOT(setIgnoreYaw(int)));

    setLayout(layout);
    setWindowTitle("Add Goal to Task");
  }

  std::string getDescription() const
  {
    return description_;
  }

  Goal::GoalType getGoalType() const
  {
    return goal_type_;
  }

  int getObject() const
  {
    return object_id_;
  }

  void setObjects(const std::vector<Object> &objects)
  {
    objects_ = objects;
    select_object_->clear();
    std::vector<Object>::iterator it;
    for(it = objects_.begin(); it != objects_.end(); ++it)
    {
      if(it->movable)
	select_object_->addItem(QString(it->label.c_str()));
    }
  }

  bool ignoreYaw() const
  {
    return ignore_yaw_;
  }

private slots:
  void goalTypeChanged(int type)
  {
    goal_type_ = (Goal::GoalType)type;
  }

  void objectChanged(int id)
  {
    int i = -1;
    std::vector<Object>::iterator it;
    for(it = objects_.begin(); it != objects_.end(); ++it)
    {
      if(it->movable)
	i++;

      if(i == id)
	break;
    }

    if(i < 0 || it == objects_.end())
    {
      ROS_ERROR("[AddGoalDialog] An error has occured in selecting the object!");
      return;
    }

    // object_id_ = objects_[id].mesh_marker_.id;
    object_id_ = it->mesh_marker_.id;
  }

  void setDescription(const QString &description)
  {
    description_ = description.toStdString();
  }

  void setIgnoreYaw(int ignore)
  {
    if(ignore == Qt::Checked)
    {
      ignore_yaw_ = true;
    }
    else if(ignore == Qt::Unchecked)
    {
      ignore_yaw_ = false;
    }
    else
    {
      ROS_ERROR("[AddGoalDialog] An error has occured in ignore yaw!");
    }
  }
  
private:
  std::string description_;
  Goal::GoalType goal_type_;
  int object_id_;
  std::vector<Object> objects_;
  bool ignore_yaw_;

  QComboBox *select_goal_type_;
  QComboBox *select_object_;

};

} // namespace demonstration_visualizer

#endif // HELPERS_H
