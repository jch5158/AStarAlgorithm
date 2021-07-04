#include "stdafx.h"
#include "CAStar.h"


CAStar::CAStar(void)
	: mMap(nullptr)
	, mMapWidth(0)
	, mMapHeight(0)
	, mDestinationNode{ 0, }
	, mOpenList()
{}

CAStar::CAStar(int mapWidth, int mapHeight)
	: mMap(nullptr)
	, mMapWidth(mapWidth)
	, mMapHeight(mapHeight)
	, mDestinationNode{ 0, }
	, mOpenList()
{
	mMap = new stNode * [mapHeight];

	for (int indexY = 0; indexY < mapHeight; ++indexY)
	{
		mMap[indexY] = new stNode[mapWidth];

		ZeroMemory(mMap[indexY], mapWidth);

		for (int indexX = 0; indexX < mapWidth; ++indexX)
		{
			mMap[indexY][indexX].x = indexX;
			mMap[indexY][indexX].y = indexY;
		}
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

	stNode* pStartNode = &mMap[startY][startX];

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
			if (setRouteArray(pOpenListNode, routeNodeArray, routeNodeArraySize) == false)
			{
				CSystemLog::GetInstance()->Log(TRUE, CSystemLog::eLogLevel::LogLevelError, L"AStar", L"[PathFind] setRouteArray, routeNodeArraySize : %d", routeNodeArraySize);

				break;
			}

			mOpenList.clear();

			resetNodeState();

			return true;
		}

		createSurroundNode(pOpenListNode);

		// node의 F 값을 기준으로 오름 차순 정렬을 한다.
		mOpenList.sort(CSortAscendingOrder());
	}

	mOpenList.clear();

	resetNodeState();

	return false;
}



bool CAStar::SetMapAttribute(int x, int y, eNodeAttribute nodeAttribute)
{
	if (x < 0 || y < 0 || x >= mMapWidth || y >= mMapHeight)
	{
		return false;
	}

	mMap[y][x].nodeAttribute = nodeAttribute;

	return true;
}

void CAStar::ResetMapAttribute(void)
{
	for (int indexY = 0; indexY < mMapHeight; ++indexY)
	{
		for (int indexX = 0; indexX < mMapWidth; ++indexX)
		{
			mMap[indexY][indexX].nodeAttribute = eNodeAttribute::NODE_UNBLOCK;
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
	if (mMap[y][x].nodeAttribute == eNodeAttribute::NODE_BLOCK || mMap[y][x].nodeState == eNodeState::NODE_CLOSED)
	{
		return;
	}

	stNode* pOpenListNode = &mMap[y][x];
	if (pOpenListNode->nodeState == eNodeState::NODE_NONE)
	{	
		pOpenListNode->pParentNode = pParentNode;

		if (abs(x - pParentNode->x) == 1 && abs(y - pParentNode->y) == 1)
		{
			pOpenListNode->G = pParentNode->G + 1.5f;
		}
		else
		{
			pOpenListNode->G = pParentNode->G + 1.0f;
		}

		pOpenListNode->H = abs(x - mDestinationNode.x) + abs(y - mDestinationNode.y);

		pOpenListNode->F = pOpenListNode->G + pOpenListNode->H;

		pOpenListNode->nodeState = eNodeState::NODE_OPENED;

		mOpenList.push_back(pOpenListNode);
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
	
	mMap[pNode->y][pNode->x].nodeState = eNodeState::NODE_CLOSED;

	return pNode;
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

bool CAStar::setRouteArray(stNode* pDestNode, stRouteNode routeNodeArray[], int routeNodeArraySize)
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



void CAStar::resetNodeState(void)
{
	for (int indexY = 0; indexY < mMapHeight; ++indexY)
	{
		for (int indexX = 0; indexX < mMapWidth; ++indexX)
		{
			mMap[indexY][indexX].nodeState = eNodeState::NODE_NONE;
		}
	}

	return;
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

			if (mMap[indexY][indexX].nodeAttribute == eNodeAttribute::NODE_BLOCK)
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

			if (mMap[indexY][indexX].nodeAttribute == eNodeAttribute::NODE_BLOCK)
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




CAStar::CSortAscendingOrder::CSortAscendingOrder(void) {}

CAStar::CSortAscendingOrder::~CSortAscendingOrder(void) {}

bool CAStar::CSortAscendingOrder::operator()(const stNode* pLeft, const stNode* pRight) const
{
	return pLeft->F < pRight->F;
}


