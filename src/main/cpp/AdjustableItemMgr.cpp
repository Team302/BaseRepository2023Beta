//====================================================================================================================================================
/// Copyright 2023 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

//C++ Includes
#include <vector>

//Team 302 Includes
#include <AdjustableItem.h>
#include <AdjustableItemMgr.h>

AdjustableItemMgr* AdjustableItemMgr::m_instance = nullptr;
AdjustableItemMgr* AdjustableItemMgr::GetInstance()
{
    if(AdjustableItemMgr::m_instance == nullptr)
    {
        AdjustableItemMgr::m_instance = new AdjustableItemMgr();
    }
    return AdjustableItemMgr::m_instance;
}

/// @brief
AdjustableItemMgr::AdjustableItemMgr() : m_adjustableItems()
{

}

void AdjustableItemMgr::RegisterAdjustableItem
(
    AdjustableItem* item
)
{
    m_adjustableItems.emplace_back(item);
}

void AdjustableItemMgr::ListenForUpdates()
{
    //check for button state changes
    //if(...)
    //{
    //PopulateNetworkTables();
    //}
}

std::vector<AdjustableItem*> AdjustableItemMgr::CheckForDifferences()
{
    std::vector<AdjustableItem*> itemsWithDiffs;
    for(auto item : m_adjustableItems)
    {
        if(item->HasDifferences())
        {
            itemsWithDiffs.emplace_back(item);
        }
    }

    return itemsWithDiffs;
}