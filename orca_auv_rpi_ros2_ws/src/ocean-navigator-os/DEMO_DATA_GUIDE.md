# Demo 資料管理系統

## 概述

所有展示模式下的資料都集中管理在 `/src/data/demoData.ts`，方便統一更新和維護。

## 資料結構

### 1. **Dashboard 資料** (`demoDashboardData`)
- **status**: 實時狀態（電池、深度、溫度、連接狀態）
- **attitude**: 3D 姿態（Pitch、Roll、Yaw）
- **videoStream**: 影片播放設置（包含影片 URL）

### 2. **Mission Manager 資料** (`demoMissionManagerData`)
- **missions**: 任務列表（名稱、狀態、動作數、持續時間）
- **history**: 執行歷史（日期、拍照數、錄影數等）

### 3. **Mission Setup 資料** (`demoMissionSetupData`)
- **targetSettings**: 目標物件識別設置
- **actionLibrary**: 可用動作列表
- **actionSequences**: 預設動作序列

### 4. **Manual Control 資料** (`demoManualControlData`)
- **joysticks**: 虛擬搖桿初始位置
- **shortcuts**: 快捷鍵配置
- **limits**: 速度、深度、距離限制

### 5. **System Settings 資料** (`demoSystemSettingsData`)
- **device**: 設備信息（名稱、序列號、固件版本）
- **communication**: 通訊設置（IP、端口、協議）
- **safety**: 安全設置（自動浮出、自動返回等）
- **notifications**: 通知設置
- **auvModels**: AUV 機型配置（4 種模型）

## 影片設置

### 存放位置
影片存放在 `/public/videos/` 目錄中

### 使用影片
1. 將 MP4 影片上傳到 `/public/videos/` 目錄
2. 在 `demoData.ts` 中更新影片 URL：
```typescript
videoStream: {
  isRecording: true,
  recordingStatus: "目標識別中...",
  videoUrl: "/videos/demo.mp4", // 更新此路徑
}
```

3. Dashboard 會在展示模式時自動播放該影片

## 使用方式

### 導入資料
```typescript
import { demoDashboardData } from "@/data/demoData";

const data = demoDashboardData;
console.log(data.status.battery); // 85
```

### 更新資料
所有資料在一個文件中管理，修改會立即影響所有使用該資料的頁面。

### 頁面整合情況
| 頁面 | 資料來源 | 狀態 |
|------|--------|------|
| Dashboard | `demoDashboardData` | ✅ 已整合 |
| Mission Manager | `demoMissionManagerData` | ✅ 已整合 |
| Mission Setup | `demoMissionSetupData` | ✅ 已整合 |
| Manual Control | `demoManualControlData` | ✅ 已整合 |
| System Settings | `demoSystemSettingsData` | ✅ 已整合 |

## 展示模式切換

在 System Settings → AUV 配置 中的「展示模式」開關可以控制：
- 啟用時：使用 demo 資料顯示
- 禁用時：連接真實 API（後續實裝）

## 影片播放

Dashboard 的「主攝影機實時畫面」會：
- **展示模式啟用 + 影片存在**：播放 `/videos/demo.mp4`
- **展示模式禁用或影片不存在**：顯示佔位符文字

## 後續擴展

如需添加新的 demo 資料：
1. 在 `demoData.ts` 中定義新的常數 (如 `demoXxxData`)
2. 在頁面中導入並使用
3. 保持資料結構一致，便於切換到真實 API

---

**最後更新**: 2025-01-17
