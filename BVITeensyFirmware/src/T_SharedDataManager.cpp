#include "T_SharedDataManager.h"

SharedDataManager::SharedDataManager()
    : data_(std::make_shared<ManagedData>()) {}

std::shared_ptr<ManagedData> SharedDataManager::getData() {
    return data_;
}
