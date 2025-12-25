# Ocean Navigator AUV API Server

完整的 Node.js/Express API 伺服器，用於 Ocean Navigator AUV 控制系統。

## 功能特性

- ✅ Dashboard 實時狀態管理
- ✅ Mission 任務建立、編輯、執行、刪除
- ✅ Mission Setup 動作配置
- ✅ Manual Control 搖桿和快捷鍵命令
- ✅ System Settings 系統設置管理
- ✅ 健康檢查端點
- ✅ CORS 支持
- ✅ TypeScript + Express

## 安裝

```bash
cd server
npm install
```

## 開發模式

```bash
npm run dev
```

伺服器將運行在 `http://localhost:3001`

## 生產構建

```bash
npm run build
npm start
```

## API 端點

### Dashboard
- `GET /api/dashboard/status` - 取得 AUV 實時狀態
- `GET /api/dashboard/attitude` - 取得 3D 姿態
- `PUT /api/dashboard/status` - 更新狀態

### Missions
- `GET /api/missions` - 取得所有任務
- `GET /api/missions/:id` - 取得單一任務
- `POST /api/missions` - 建立新任務
- `PUT /api/missions/:id` - 更新任務
- `DELETE /api/missions/:id` - 刪除任務
- `POST /api/missions/:id/execute` - 執行任務

### Mission Setup
- `GET /api/mission-setup/actions` - 取得可用動作列表
- `POST /api/mission-setup/save` - 保存任務設置

### Manual Control
- `POST /api/manual-control/joystick` - 控制搖桿
- `POST /api/manual-control/shortcut` - 執行快捷鍵命令

### System Settings
- `GET /api/system-settings` - 取得系統設置
- `PUT /api/system-settings` - 更新系統設置
- `GET /api/system-settings/execution-history` - 取得執行歷史

### 其他
- `GET /api/health` - 健康檢查
- `GET /` - 根路由（列出所有端點）

## 環境變數

複製 `.env.example` 為 `.env` 並修改：

```env
PORT=3001
NODE_ENV=development
```

## 前端整合

在前端應用中配置 API 基礎 URL：

```typescript
const API_BASE_URL = 'http://localhost:3001/api';

// 或環境變數
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:3001/api';
```

然後使用 `@tanstack/react-query` 或 `axios` 調用 API：

```typescript
const fetchStatus = async () => {
  const response = await fetch(`${API_BASE_URL}/dashboard/status`);
  return response.json();
};
```

## 部署指南

### 使用 PM2

```bash
npm install -g pm2
pm2 start dist/index.js --name "ocean-navigator-api"
pm2 save
pm2 startup
```

### 使用 Docker

```dockerfile
FROM node:18-alpine
WORKDIR /app
COPY package*.json ./
RUN npm ci --only=production
COPY dist ./dist
EXPOSE 3001
CMD ["npm", "start"]
```

### 使用 Heroku

```bash
heroku create ocean-navigator-api
git push heroku main
```

## 資料庫整合

目前使用內存存儲。生產環境應集成真實資料庫：

- MongoDB（推薦用於 Node.js）
- PostgreSQL
- MySQL

在 `index.ts` 中替換內存變數為資料庫查詢即可。

## 後續開發

- [ ] 資料庫持久化（MongoDB/PostgreSQL）
- [ ] 身份驗證（JWT）
- [ ] WebSocket 實時數據推送
- [ ] 檔案上傳（照片/影片）
- [ ] API 文件（Swagger）
- [ ] 單元測試
- [ ] 日誌系統
- [ ] 監控告警

---

**API 版本**: 1.0.0  
**最後更新**: 2025-01-17
