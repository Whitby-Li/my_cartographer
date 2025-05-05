//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_TRAJECTORY_CONNECTIVITY_STATE_H_
#define MY_CARTOGRAPHER_MAP_INTERNAL_TRAJECTORY_CONNECTIVITY_STATE_H_

#include "my_cartographer/common/time.h"
#include "my_cartographer/map/internal/connected_components.h"

namespace my_cartographer
{
  namespace map
  {

    // A class that tracks the connectivity state between trajectories. Compared to
    // ConnectedComponents it tracks additionally the last time that a global
    // constraint connected to trajectories.
    //
    // This class is thread-compatible.
    class TrajectoryConnectivityState
    {
    public:
      TrajectoryConnectivityState() {}

      TrajectoryConnectivityState(const TrajectoryConnectivityState &) = delete;
      TrajectoryConnectivityState &operator=(const TrajectoryConnectivityState &) =
          delete;

      // Add a trajectory which is initially connected to only itself.
      void Add(int trajectory_id);

      // Connect two trajectories. If either trajectory is untracked, it will be
      // tracked. This function is invariant to the order of its arguments. Repeated
      // calls to Connect increment the connectivity count and update the last
      // connected time.
      void Connect(int trajectory_id_a, int trajectory_id_b, common::Time time);

      // Determines if two trajectories have been (transitively) connected. If
      // either trajectory is not being tracked, returns false, except when it is
      // the same trajectory, where it returns true. This function is invariant to
      // the order of its arguments.
      bool TransitivelyConnected(int trajectory_id_a, int trajectory_id_b) const;

      // The trajectory IDs, grouped by connectivity.
      std::vector<std::vector<int>> Components() const;

      // Return the last connection count between the two trajectories. If either of
      // the trajectories is untracked or they have never been connected returns the
      // beginning of time.
      common::Time LastConnectionTime(int trajectory_id_a, int trajectory_id_b);

    private:
      // ConnectedComponents are thread safe.
      mutable ConnectedComponents connected_components_;

      // Tracks the last time a direct connection between two trajectories has
      // been added. The exception is when a connection between two trajectories
      // connects two formerly unconnected connected components. In this case all
      // bipartite trajectories entries for these components are updated with the
      // new connection time.
      std::map<std::pair<int, int>, common::Time> last_connection_time_map_;
    };

  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_TRAJECTORY_CONNECTIVITY_STATE_H_