#pragma once

class ClientClass;

class IBaseClientDLL {
	public:
		ClientClass* GetAllClasses() {
			return GetVirtualFunction<ClientClass*(*)(IBaseClientDLL*)>(this, 8)(this);
		}
};

extern IBaseClientDLL* clientdll;