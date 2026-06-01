#include "T_SharedDataManagerClass.h"

SharedDataManager::SharedDataManager() {
	data = std::make_shared<ManagedData>();
}

std::shared_ptr<ManagedData> SharedDataManager::getData() {
	return data;
}


/**
 * @brief Load text into debug terminal
 * 
 * @param msg 
 */
void ManagedData::PrintDebug ( const String& msg ) {
	
	if ( serialClassPtr ) { 
		serialClassPtr->PrintDebug(msg) ;
	}
	

}