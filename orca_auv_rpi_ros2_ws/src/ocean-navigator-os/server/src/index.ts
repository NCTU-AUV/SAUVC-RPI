import express, { Express, Request, Response } from 'express';
import cors from 'cors';
import dotenv from 'dotenv';
import { demoSimulator } from './demoSimulator';

dotenv.config();

const app: Express = express();
const PORT = process.env.PORT || 3001;

// 讀取環境變數，判斷是否啟用 demo 模式（預設為 true）
const DEMO_MODE = process.env.DEMO_MODE !== 'false';

// Middleware
app.use(cors());
app.use(express.json());

// ==================== Types ====================
interface AUVStatus {
  battery: number;
  depth: number;
  pressure: number;
  temperature: number;
  connection: string;
  latency: number;
  bandwidth: string;
}

interface Attitude {
  pitch: number;
  roll: number;
  yaw: number;
}

interface Mission {
  id: number;
  name: string;
  created: string;
  actions: number;
  duration: string;
  status: 'completed' | 'executing' | 'draft';
}

// ==================== In-Memory Database ====================

let auvStatus: AUVStatus = {
  battery: 85,
  depth: 45.2,
  pressure: 5.52,
  temperature: 18.5,
  connection: 'connected',
  latency: 45,
  bandwidth: 'good'
};

let attitude: Attitude = {
  pitch: 5.2,
  roll: -2.1,
  yaw: 180.0
};

const missions: Mission[] = [
  {
    id: 1,
    name: '珊瑚礁調查任務',
    created: '2024-01-15',
    actions: 12,
    duration: '45分鐘',
    status: 'completed'
  },
  {
    id: 2,
    name: '水下管道檢測',
    created: '2024-01-14',
    actions: 8,
    duration: '30分鐘',
    status: 'executing'
  },
  {
    id: 3,
    name: '海底地形測繪',
    created: '2024-01-13',
    actions: 15,
    duration: '60分鐘',
    status: 'draft'
  }
];

interface ExecutionRecord {
  id: number;
  name: string;
  date: string;
  duration: string;
  targets: number;
  photos: number;
  videos: number;
}

const executionHistory: ExecutionRecord[] = [
  {
    id: 1,
    name: '珊瑚礁調查任務',
    date: '2024-01-15 14:30',
    duration: '42分鐘',
    targets: 8,
    photos: 45,
    videos: 3
  }
];

// ==================== Dashboard Routes ====================

/**
 * GET /api/dashboard/status
 * 取得 AUV 實時狀態
 */
app.get('/api/dashboard/status', (req: Request, res: Response) => {
  if (DEMO_MODE) {
    // 在 demo 模式中使用動態數據
    const demoData = demoSimulator.getFullData();
    return res.json({
      success: true,
      data: demoData.status,
      demo: true
    });
  }
  
  res.json({
    success: true,
    data: auvStatus,
    demo: false
  });
});

/**
 * GET /api/dashboard/attitude
 * 取得 AUV 3D 姿態
 */
app.get('/api/dashboard/attitude', (req: Request, res: Response) => {
  if (DEMO_MODE) {
    // 在 demo 模式中使用動態數據
    const demoData = demoSimulator.getFullData();
    return res.json({
      success: true,
      data: demoData.attitude,
      demo: true
    });
  }

  res.json({
    success: true,
    data: attitude,
    demo: false
  });
});

/**
 * GET /api/dashboard/thrusters
 * 取得推進器狀態
 */
app.get('/api/dashboard/thrusters', (req: Request, res: Response) => {
  if (DEMO_MODE) {
    const demoData = demoSimulator.getFullData();
    const thrusters = demoData.thrusters.map((speed, index) => ({
      id: index + 1,
      name: `推進器 ${index + 1}`,
      speed,
      rpm: speed
    }));
    return res.json({
      success: true,
      data: thrusters,
      demo: true
    });
  }

  res.json({
    success: true,
    data: [],
    demo: false
  });
});

/**
 * GET /api/dashboard/task-progress
 * 取得任務進度
 */
app.get('/api/dashboard/task-progress', (req: Request, res: Response) => {
  if (DEMO_MODE) {
    const demoData = demoSimulator.getFullData();
    return res.json({
      success: true,
      data: {
        action: '前進至目標區域',
        progress: demoData.taskProgress.overall,
        completed: Math.floor(demoData.taskProgress.overall / 12.5),
        total: 8,
        targets: demoData.taskProgress.targets,
        photos: demoData.taskProgress.photos,
        taskTime: demoData.taskProgress.taskTime
      },
      demo: true
    });
  }

  res.json({
    success: true,
    data: {
      action: '待執行',
      progress: 0,
      completed: 0,
      total: 0,
      targets: 0,
      photos: 0,
      taskTime: '00:00:00'
    },
    demo: false
  });
});

/**
 * PUT /api/dashboard/status
 * 更新 AUV 狀態（僅在非 demo 模式生效）
 */
app.put('/api/dashboard/status', (req: Request, res: Response) => {
  if (DEMO_MODE) {
    return res.status(400).json({
      success: false,
      error: 'Demo 模式下無法修改數據'
    });
  }

  try {
    const { battery, depth, pressure, temperature, latency } = req.body;
    
    if (battery !== undefined) auvStatus.battery = battery;
    if (depth !== undefined) auvStatus.depth = depth;
    if (pressure !== undefined) auvStatus.pressure = pressure;
    if (temperature !== undefined) auvStatus.temperature = temperature;
    if (latency !== undefined) auvStatus.latency = latency;

    res.json({
      success: true,
      message: '狀態已更新',
      data: auvStatus
    });
  } catch (error) {
    res.status(400).json({
      success: false,
      error: '更新失敗'
    });
  }
});

// ==================== Mission Manager Routes ====================

/**
 * GET /api/missions
 * 取得所有任務
 */
app.get('/api/missions', (req: Request, res: Response) => {
  if (DEMO_MODE) {
    // 在 demo 模式中增加動態任務進度
    const demoData = demoSimulator.getFullData();
    const missionsWithProgress = missions.map((mission, index) => ({
      ...mission,
      progress: index === 1 ? demoData.taskProgress.overall : 0 // 第二個任務在執行中
    }));
    return res.json({
      success: true,
      data: missionsWithProgress,
      demo: true
    });
  }

  res.json({
    success: true,
    data: missions,
    demo: false
  });
});

/**
 * GET /api/missions/:id
 * 取得單一任務詳情
 */
app.get('/api/missions/:id', (req: Request, res: Response) => {
  const mission = missions.find(m => m.id === parseInt(req.params.id));
  
  if (!mission) {
    return res.status(404).json({
      success: false,
      error: '任務不存在'
    });
  }

  res.json({
    success: true,
    data: mission
  });
});

/**
 * POST /api/missions
 * 建立新任務
 */
app.post('/api/missions', (req: Request, res: Response) => {
  if (DEMO_MODE) {
    return res.status(400).json({
      success: false,
      error: 'Demo 模式下無法建立任務'
    });
  }

  try {
    const { name, duration } = req.body;

    if (!name) {
      return res.status(400).json({
        success: false,
        error: '任務名稱為必填'
      });
    }

    const newMission: Mission = {
      id: Math.max(...missions.map(m => m.id), 0) + 1,
      name,
      created: new Date().toISOString().split('T')[0],
      actions: 0,
      duration: duration || '未知',
      status: 'draft'
    };

    missions.push(newMission);

    res.status(201).json({
      success: true,
      message: '任務已建立',
      data: newMission
    });
  } catch (error) {
    res.status(400).json({
      success: false,
      error: '建立失敗'
    });
  }
});

/**
 * PUT /api/missions/:id
 * 更新任務
 */
app.put('/api/missions/:id', (req: Request, res: Response) => {
  if (DEMO_MODE) {
    return res.status(400).json({
      success: false,
      error: 'Demo 模式下無法修改任務'
    });
  }

  try {
    const mission = missions.find(m => m.id === parseInt(req.params.id));

    if (!mission) {
      return res.status(404).json({
        success: false,
        error: '任務不存在'
      });
    }

    const { name, duration, status, actions } = req.body;

    if (name !== undefined) mission.name = name;
    if (duration !== undefined) mission.duration = duration;
    if (status !== undefined) mission.status = status;
    if (actions !== undefined) mission.actions = actions;

    res.json({
      success: true,
      message: '任務已更新',
      data: mission
    });
  } catch (error) {
    res.status(400).json({
      success: false,
      error: '更新失敗'
    });
  }
});

/**
 * DELETE /api/missions/:id
 * 刪除任務
 */
app.delete('/api/missions/:id', (req: Request, res: Response) => {
  if (DEMO_MODE) {
    return res.status(400).json({
      success: false,
      error: 'Demo 模式下無法刪除任務'
    });
  }

  const index = missions.findIndex(m => m.id === parseInt(req.params.id));

  if (index === -1) {
    return res.status(404).json({
      success: false,
      error: '任務不存在'
    });
  }

  const deletedMission = missions.splice(index, 1);

  res.json({
    success: true,
    message: '任務已刪除',
    data: deletedMission[0]
  });
});

/**
 * POST /api/missions/:id/execute
 * 執行任務
 */
app.post('/api/missions/:id/execute', (req: Request, res: Response) => {
  const mission = missions.find(m => m.id === parseInt(req.params.id));

  if (!mission) {
    return res.status(404).json({
      success: false,
      error: '任務不存在'
    });
  }

  mission.status = 'executing';

  res.json({
    success: true,
    message: `任務 "${mission.name}" 已開始執行`,
    data: mission
  });
});

// ==================== Mission Setup Routes ====================

/**
 * GET /api/mission-setup/actions
 * 取得可用動作列表
 */
app.get('/api/mission-setup/actions', (req: Request, res: Response) => {
  const actions = [
    { id: 'approach', name: '靠近目標', icon: '🎯' },
    { id: 'circle', name: '環繞掃描', icon: '⭕' },
    { id: 'photo', name: '拍攝照片', icon: '📷' },
    { id: 'sample', name: '採集樣本', icon: '🧬' },
    { id: 'measure', name: '測量距離', icon: '📏' },
    { id: 'sonar', name: '聲納掃描', icon: '📡' },
    { id: 'video', name: '開始錄影', icon: '🎥' },
    { id: 'ascend', name: '上升', icon: '⬆️' }
  ];

  res.json({
    success: true,
    data: actions
  });
});

/**
 * POST /api/mission-setup/save
 * 保存任務設置
 */
app.post('/api/mission-setup/save', (req: Request, res: Response) => {
  try {
    const { missionId, targetType, confidence, actionSequence } = req.body;

    if (!missionId) {
      return res.status(400).json({
        success: false,
        error: '任務 ID 為必填'
      });
    }

    res.json({
      success: true,
      message: '任務設置已保存',
      data: {
        missionId,
        targetType,
        confidence,
        actions: actionSequence?.length || 0
      }
    });
  } catch (error) {
    res.status(400).json({
      success: false,
      error: '保存失敗'
    });
  }
});

// ==================== Manual Control Routes ====================

/**
 * POST /api/manual-control/joystick
 * 控制虛擬搖桿
 */
app.post('/api/manual-control/joystick', (req: Request, res: Response) => {
  try {
    const { leftStick, rightStick } = req.body;

    res.json({
      success: true,
      message: '搖桿命令已發送',
      data: {
        leftStick,
        rightStick
      }
    });
  } catch (error) {
    res.status(400).json({
      success: false,
      error: '命令發送失敗'
    });
  }
});

/**
 * POST /api/manual-control/shortcut
 * 執行快捷鍵命令
 */
app.post('/api/manual-control/shortcut', (req: Request, res: Response) => {
  try {
    const { key, action } = req.body;

    if (!key || !action) {
      return res.status(400).json({
        success: false,
        error: '快捷鍵和動作為必填'
      });
    }

    res.json({
      success: true,
      message: `快捷鍵 ${key} 已執行：${action}`,
      data: { key, action }
    });
  } catch (error) {
    res.status(400).json({
      success: false,
      error: '命令執行失敗'
    });
  }
});

// ==================== System Settings Routes ====================

/**
 * GET /api/system-settings
 * 取得系統設置
 */
app.get('/api/system-settings', (req: Request, res: Response) => {
  const settings = {
    device: {
      name: 'AUV-001',
      serialNumber: 'AUV-2024-001-SN12345',
      firmwareVersion: 'v2.5.1'
    },
    communication: {
      ipAddress: '192.168.1.100',
      port: '8080',
      protocol: 'TCP/IP',
      bandwidth: 2.5
    },
    safety: {
      autoSurface: true,
      autoSurfaceDepth: 100,
      autoReturn: true,
      autoReturnDistance: 1500,
      collisionDetection: true,
      obstacleDistance: 2.0
    },
    auvModels: {
      small: { name: '小型 AUV', maxDepth: 20, maxDistance: 200, maxSpeed: 1.5, battery: 4 },
      medium: { name: '中型 AUV', maxDepth: 100, maxDistance: 500, maxSpeed: 2.0, battery: 8 },
      large: { name: '大型 AUV', maxDepth: 500, maxDistance: 2000, maxSpeed: 2.5, battery: 12 },
      deep: { name: '深海 AUV', maxDepth: 6000, maxDistance: 5000, maxSpeed: 1.5, battery: 16 }
    }
  };

  res.json({
    success: true,
    data: settings
  });
});

/**
 * PUT /api/system-settings
 * 更新系統設置
 */
app.put('/api/system-settings', (req: Request, res: Response) => {
  try {
    const { device, communication, safety } = req.body;

    res.json({
      success: true,
      message: '系統設置已更新',
      data: {
        device,
        communication,
        safety
      }
    });
  } catch (error) {
    res.status(400).json({
      success: false,
      error: '更新失敗'
    });
  }
});

/**
 * GET /api/system-settings/execution-history
 * 取得任務執行歷史
 */
app.get('/api/system-settings/execution-history', (req: Request, res: Response) => {
  res.json({
    success: true,
    data: executionHistory
  });
});

// ==================== Health Check ====================

/**
 * GET /api/health
 * 健康檢查
 */
app.get('/api/health', (req: Request, res: Response) => {
  res.json({
    success: true,
    message: 'API Server 正常運行',
    timestamp: new Date().toISOString(),
    uptime: process.uptime(),
    demoMode: DEMO_MODE
  });
});

/**
 * GET /
 * 根路徑
 */
app.get('/', (req: Request, res: Response) => {
  res.json({
    success: true,
    message: 'Ocean Navigator AUV API Server v1.0.0',
    demoMode: DEMO_MODE,
    endpoints: {
      health: '/api/health',
      dashboard: '/api/dashboard/*',
      missions: '/api/missions*',
      missionSetup: '/api/mission-setup/*',
      manualControl: '/api/manual-control/*',
      systemSettings: '/api/system-settings*'
    }
  });
});

// ==================== Error Handling ====================

app.use((req: Request, res: Response) => {
  res.status(404).json({
    success: false,
    error: '找不到該路由'
  });
});

// ==================== Start Server ====================

app.listen(PORT, () => {
  console.log(`🚀 API Server 運行於 http://localhost:${PORT}`);
  console.log(`📝 健康檢查：http://localhost:${PORT}/api/health`);
  console.log(`📚 API 文件：http://localhost:${PORT}/`);
  console.log(`🎯 Demo 模式：${DEMO_MODE ? '✅ 啟用' : '❌ 禁用'}`);
});

export default app;
