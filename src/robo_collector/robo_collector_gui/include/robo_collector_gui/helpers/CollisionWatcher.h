#ifndef ROBO_COLLECTOR_GUI_COLLISIONWATCHER_H_
#define ROBO_COLLECTOR_GUI_COLLISIONWATCHER_H_

//C system headers

//C++ system headers
#include <vector>

//Other libraries headers

//Own components headers
#include "robo_collector_gui/helpers/CollisionObject.h"

//Forward declarations

class CollisionWatcher {
public:
  CollisionObjHandle registerObject(CollisionObject* object);
  void unregisterObject(CollisionObjHandle handle);

  void toggleWatchStatus(CollisionObjHandle handle,
                         CollisionWatchStatus status);

  //checks all _activeWatchedObjects against all objects
  void process();

private:
  std::vector<CollisionObject*> _objects;

  std::vector<CollisionObjHandle> _activeWatchedHandles;
};

#endif /* ROBO_COLLECTOR_GUI_COLLISIONWATCHER_H_ */
