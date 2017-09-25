rostopic pub -1 /tb00/move_base/goal move_base_msgs/MoveBaseActionGoal \
'{header: { seq: 0, stamp: 0, frame_id: "/map"},
  goal_id: {
    stamp: 0,
    id: "map"
  },
  goal: {
    target_pose: {
      header: {
        seq: 0,
        stamp: 0,
        frame_id: "/map"
      },
      pose:{
        position:{
          x: 1.5,
          y: 0.,
          z: 0.
        },
        orientation:{
          x: 0.,
          y: 0.,
          z: 0.,
          w: 1.
        }
      }
    }
  }
}'
