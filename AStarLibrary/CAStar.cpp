#include "stdafx.h"
#include "CAStar.h"


CAStar::CAStar(void)
	: mMap(nullptr)
	, mMapWidth(0)
	, mMapHeight(0)
	, mDestinationNode{ 0, }
	, mOpenList()
	, mClosedList()
{}

CAStar::CAStar(int mapWidth, int mapHeight)
	: mMap(nullptr)
	, mMapWidth(mapWidth)
	, mMapHeight(mapHeight)
	, mDestinationNode{ 0, }
	, mOpenList()
	, mClosedList()
{
	mMap = new char* [mapHeight];

	for (int indexY = 0; indexY < mapHeight; ++indexY)
	{
		mMap[indexY] = new char[mapWidth];	

		ZeroMemory(mMap[indexY], mapWidth);
	}
}

CAStar::~CAStar(void)
{
	for (int indexY = 0; indexY < mMapHeight; ++indexY)
	{
		delete[] mMap[indexY];
	}

	delete[] mMap;
}


bool CAStar::PathFind(int startX, int startY, int destinationX, int destinationY, stRouteNode routeNodeArray[], int routeNodeArraySize)
{
	if (startX < 0 || startY < 0 || startX >= mMapWidth || startY >= mMapHeight)
	{
		return false;
	}

	stNode* pStartNode = (stNode*)malloc(sizeof(stNode));
	if (pStartNode == nullptr)
	{
		CSystemLog::GetInstance()->Log(true, CSystemLog::eLogLevel::LogLevelError, L"AStar", L"[PathFind] malloc is nullptr return : %d", GetLastError());

		CCrashDump::Crash();

		return false;
	}

	pStartNode->x = startX;
	pStartNode->y = startY;
	pStartNode->pParentNode = nullptr;
	pStartNode->G = 0.0f;
	pStartNode->H = (float)(abs(startX - destinationX) + abs(startY - destinationY));
	pStartNode->F = pStartNode->G + pStartNode->H;


	// 도착지 노드 셋팅
	mDestinationNode.x = destinationX;
	mDestinationNode.y = destinationY;

	mOpenList.push_back(pStartNode);

	for (;;)
	{
		stNode* pOpenListNode = getExplorationNodeFromOpenList();
		if (pOpenListNode == nullptr)
		{
			break;
		}
		else if (pOpenListNode->x == destinationX && pOpenListNode->y == destinationY)
		{
			for (int index = 0; index < routeNodeArraySize; ++index)
			{
				if (setRouteArray(pOpenListNode, routeNodeArray, routeNodeArraySize) == false)
				{
					clearOpenList();

					clearClosedList();

					return false;
				}
			}

			clearOpenList();

			clearClosedList();

			return true;
		}

		createSurroundNode(pOpenListNode);

		// node의 F 값을 기준으로 오름 차순 정렬을 한다.
		mOpenList.sort(CSortAscendingOrder());
	}

	clearOpenList();

	clearClosedList();

	return false;
}



bool CAStar::SetMapAttribute(int x, int y, eNodeAttribute nodeAttribute)
{
	if (x < 0 || y < 0 || x >= mMapWidth || y >= mMapHeight)
	{
		return false;
	}

	mMap[y][x] = (char)nodeAttribute;
		
	return true;
}

void CAStar::ResetMapAttribute(void)
{
	for (int indexY = 0; indexY < mMapHeight; ++indexY)
	{
		for (int indexX = 0; indexX < mMapWidth; ++indexX)
		{
			mMap[indexY][indexX] = (char)eNodeAttribute::NODE_UNBLOCK;
		}
	}

	return;
}


void CAStar::createSurroundNode(stNode* pNode)
{
	int x = pNode->x - 1;
	int y = pNode->y - 1;

	for (int indexY = 0; indexY < 3; ++indexY)
	{
		if (y + indexY < 0 || y + indexY >= mMapHeight)
		{
			continue;
		}

		for (int indexX = 0; indexX < 3; ++indexX)
		{
			if (x + indexX < 0 || x + indexX >= mMapWidth)
			{
				continue;
			}

			createNode(x + indexX, y + indexY, pNode);
		}
	}

	return;
}

void CAStar::createNode(int x, int y, stNode* pParentNode)
{
	if (mMap[y][x] == (char)eNodeAttribute::NODE_BLOCK || findClosedListNode(x, y) != nullptr)
	{
		return;
	}

	stNode* pOpenListNode = findOpenListNode(x, y);
	if (pOpenListNode == nullptr)
	{
		stNode* pNewNode = (stNode*)malloc(sizeof(stNode));
		if (pNewNode == nullptr)
		{
			CSystemLog::GetInstance()->Log(TRUE, CSystemLog::eLogLevel::LogLevelError, L"AStar", L"[createNode] Error Code : %d", GetLastError());

			CCrashDump::Crash();

			return;
		}	

		pNewNode->x = x;
		pNewNode->y = y;
		pNewNode->pParentNode = pParentNode;

		if (abs(x - pParentNode->x) == 1 && abs(y - pParentNode->y) == 1)
		{
			pNewNode->G = pParentNode->G + 1.5f;
		}
		else
		{
			pNewNode->G = pParentNode->G + 1.0f;
		}

		pNewNode->H = abs(x - mDestinationNode.x) + abs(y - mDestinationNode.y);

		pNewNode->F = pNewNode->G + pNewNode->H;
	
		mOpenList.push_back(pNewNode);
	}
	else
	{
		if (pOpenListNode->pParentNode->G > pParentNode->G)
		{
			if (abs(pOpenListNode->x - pParentNode->x) == 1 && abs(pOpenListNode->y - pParentNode->y) == 1)
			{
				pOpenListNode->G = pParentNode->G + 1.5f;
			}
			else
			{
				pOpenListNode->G = pParentNode->G + 1.0f;
			}

			pOpenListNode->F = pOpenListNode->G + pOpenListNode->H;

			pOpenListNode->pParentNode = pParentNode;
		}
	}

	return;
}

CAStar::stNode* CAStar::getExplorationNodeFromOpenList(void)
{
	auto iter = mOpenList.begin();
	if (iter == mOpenList.end())
	{
		return nullptr;
	}

	stNode* pNode = *iter;

	// openList에서 제거한다.
	mOpenList.erase(iter);

	// closedList에 추가한다.
	mClosedList.push_back(pNode);

	return pNode;
}

CAStar::stNode* CAStar::findClosedListNode(int x, int y)
{
	for (stNode* pNode : mClosedList)
	{
		if (pNode->x == x && pNode->y == y)
		{
			return pNode;	
		}
	}

	return nullptr;
}


CAStar::stNode* CAStar::findOpenListNode(int x, int y)
{
	for (stNode* pNode : mOpenList)
	{
		if (pNode->x == x && pNode->y == y)
		{
			return pNode;	
		}
	}

	return nullptr;
}

bool CAStar::setRouteArray(stNode* pDestNode,stRouteNode routeNodeArray[], int routeNodeArraySize)
{
	makeOptimizePath(pDestNode);

	int nodeCount = 0;

	stNode* pTempNode = pDestNode;

	for (;;)
	{
		pTempNode = pTempNode->pParentNode;

		if (pTempNode == nullptr)
		{
			break;
		}

		++nodeCount;
	}

	if (nodeCount > routeNodeArraySize)
	{
		return false;
	}

	for (int index = nodeCount; index >= 0; --index)
	{
		routeNodeArray[index].bUseFlag = true;
		routeNodeArray[index].x = pDestNode->x;
		routeNodeArray[index].y = pDestNode->y;

		pDestNode = pDestNode->pParentNode;
	}

	return true;
}


bool CAStar::makeBresenhamLine(int startX, int startY, int endX, int endY)
{
	int subX = abs(startX - endX);
	int subY = abs(startY - endY);

	int indexX;
	int indexY;

	int errorValue;

	int addX = 0;
	int addY = 0;

	if (startX <= endX)
	{
		indexX = startX + addX;
	}
	else
	{
		indexX = startX - addX;
	}

	if (startY <= endY)
	{
		indexY = startY + addY;
	}
	else
	{
		indexY = startY - addY;
	}

	if (subX >= subY)
	{
		errorValue = subX / 2;

		for (;;)
		{
			if (subX == addX && subY == addY)
			{
				break;
			}

			if (mMap[indexY][indexX] == (char)eNodeAttribute::NODE_BLOCK)
			{
				return false;
			}

			addX += 1;
			if (startX <= endX)
			{
				indexX = startX + addX;
			}
			else
			{
				indexX = startX - addX;
			}

			errorValue += subY;
			if (subX <= errorValue)
			{
				addY += 1;
				if (startY <= endY)
				{
					indexY = startY + addY;
				}
				else
				{
					indexY = startY - addY;
				}

				errorValue -= subX;
			}
		}
	}
	else
	{
		errorValue = subY / 2;

		for (;;)
		{
			if (subX == addX && subY == addY)
			{
				break;
			}

			if (mMap[indexY][indexX] == (char)eNodeAttribute::NODE_BLOCK)
			{
				return false;
			}

			addY += 1;
			if (startY <= endY)
			{
				indexY = startY + addY;
			}
			else
			{
				indexY = startY - addY;
			}

			errorValue += subX;
			if (subY <= errorValue)
			{
				addX += 1;
				if (startX <= endX)
				{
					indexX = startX + addX;
				}
				else
				{
					indexX = startX - addX;
				}

				errorValue -= subY;
			}
		}
	}

	return true;
}

void CAStar::makeOptimizePath(stNode* pNode)
{
	int startX;
	int startY;

	int endX;
	int endY;

	for (;;)
	{
		stNode* pNextNode = pNode->pParentNode;

		if (pNextNode == nullptr)
		{
			return;
		}

		startX = pNode->x;
		startY = pNode->y;

		for (;;)
		{
			stNode* pNextNextNode = pNextNode->pParentNode;

			if (pNextNextNode == nullptr)
			{
				return;
			}

			endX = pNextNextNode->x;
			endY = pNextNextNode->y;

			if (makeBresenhamLine(startX, startY, endX, endY) == true)
			{
				pNode->pParentNode = pNextNextNode;

				pNextNode = pNextNextNode;

				if (pNextNode == nullptr)
				{
					return;
				}
			}
			else
			{
				pNode = pNextNode;

				break;
			}
		}
	}

	return;
}


void CAStar::clearOpenList(void)
{
	auto iterE = mOpenList.end();

	for (auto iter = mOpenList.begin(); iter != iterE; )
	{
		free(*iter);
	
		iter = mOpenList.erase(iter);
	}

	return;
}


void CAStar::clearClosedList(void)
{
	auto iterE = mClosedList.end();

	for (auto iter = mClosedList.begin(); iter != iterE;)
	{
		free(*iter);

		iter = mClosedList.erase(iter);
	}

	return;
}


CAStar::CSortAscendingOrder::CSortAscendingOrder(void){}

CAStar::CSortAscendingOrder::~CSortAscendingOrder(void){}

bool CAStar::CSortAscendingOrder::operator()(const stNode* pLeft, const stNode* pRight) const
{
	return pLeft->F < pRight->F;
}


