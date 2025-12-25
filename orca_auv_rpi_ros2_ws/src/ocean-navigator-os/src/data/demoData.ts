// Demo 資料管理系統 - 包含所有頁面的展示資料

export const demoDashboardData = {
  // 實時狀態數據
  status: {
    battery: 85,
    batteryRemaining: "2小時 15分",
    depth: 45.2,
    pressure: 5.52,
    temperature: 18.5,
    connection: "已連接",
    latency: 45,
    bandwidth: "良好",
  },

  // 3D 姿態數據
  attitude: {
    pitch: 5.2,
    roll: -2.1,
    yaw: 180.0,
  },

  // 實時影片 (監視器畫面)
  videoStream: {
    isRecording: true,
    recordingStatus: "目標識別中...",
    videoUrl: "/videos/demo.mp4", // 你上傳的 MP4 影片路徑
  },
};

export const demoMissionManagerData = {
  // 任務列表
  missions: [
    {
      id: 1,
      name: "珊瑚礁調查任務",
      created: "2024-01-15",
      actions: 12,
      duration: "45分鐘",
      status: "已完成",
    },
    {
      id: 2,
      name: "水下管道檢測",
      created: "2024-01-14",
      actions: 8,
      duration: "30分鐘",
      status: "執行中",
    },
    {
      id: 3,
      name: "海底地形測繪",
      created: "2024-01-13",
      actions: 15,
      duration: "60分鐘",
      status: "草稿",
    },
    {
      id: 4,
      name: "海洋生物觀測",
      created: "2024-01-12",
      actions: 10,
      duration: "50分鐘",
      status: "已完成",
    },
  ],

  // 執行歷史
  history: [
    {
      id: 1,
      name: "珊瑚礁調查任務",
      date: "2024-01-15 14:30",
      duration: "42分鐘",
      targets: 8,
      photos: 45,
      videos: 3,
    },
    {
      id: 2,
      name: "海洋生物觀測",
      date: "2024-01-12 09:15",
      duration: "38分鐘",
      targets: 12,
      photos: 67,
      videos: 5,
    },
    {
      id: 3,
      name: "水下管道檢測",
      date: "2024-01-10 16:45",
      duration: "28分鐘",
      targets: 5,
      photos: 32,
      videos: 2,
    },
  ],
};

export const demoMissionSetupData = {
  // 目標物件識別設置
  targetSettings: {
    objectType: "珊瑚",
    detectionThreshold: 75,
    trackingEnabled: true,
    recordingEnabled: true,
  },

  // 預設動作庫
  actionLibrary: [
    { id: "approach", name: "靠近目標", icon: "🎯" },
    { id: "circle", name: "環繞掃描", icon: "⭕" },
    { id: "photo", name: "拍攝照片", icon: "📷" },
    { id: "sample", name: "採集樣本", icon: "🧬" },
    { id: "measure", name: "測量距離", icon: "📏" },
    { id: "sonar", name: "聲納掃描", icon: "📡" },
    { id: "video", name: "開始錄影", icon: "🎥" },
    { id: "ascend", name: "上升", icon: "⬆️" },
  ],

  // 預設動作序列
  actionSequences: [
    {
      id: "coral-survey",
      name: "珊瑚礁調查序列",
      actions: [
        { id: "1", name: "靠近目標" },
        { id: "2", name: "環繞掃描" },
        { id: "3", name: "拍攝照片" },
        { id: "4", name: "測量距離" },
      ],
    },
    {
      id: "pipe-inspection",
      name: "管道檢測序列",
      actions: [
        { id: "1", name: "靠近目標" },
        { id: "2", name: "開始錄影" },
        { id: "3", name: "聲納掃描" },
        { id: "4", name: "上升" },
      ],
    },
  ],
};

export const demoManualControlData = {
  // 虛擬搖桿初始位置
  joysticks: {
    leftStick: { x: 0, y: 0 }, // 前進/轉向
    rightStick: { x: 0, y: 0 }, // 上升/下降
  },

  // 快捷鍵對應
  shortcuts: [
    { key: "C", action: "相機", emoji: "📷" },
    { key: "V", action: "錄影", emoji: "🎥" },
    { key: "L", action: "燈光", emoji: "💡" },
    { key: "X", action: "採集", emoji: "🧬" },
    { key: "Z", action: "返回", emoji: "🏠" },
  ],

  // 速度/深度限制
  limits: {
    maxSpeed: 2.5,
    maxDepth: 500,
    maxDistance: 2000,
  },
};

export const demoSystemSettingsData = {
  // 設備信息
  device: {
    name: "AUV-001",
    serialNumber: "AUV-2024-001-SN12345",
    firmwareVersion: "v2.5.1",
  },

  // 通訊設置
  communication: {
    ipAddress: "192.168.1.100",
    port: "8080",
    protocol: "TCP/IP",
    bandwidth: 2.5,
  },

  // 安全設置
  safety: {
    autoSurface: true,
    autoSurfaceDepth: 100,
    autoReturn: true,
    autoReturnDistance: 1500,
    collisionDetection: true,
    obstacleDistance: 2.0,
  },

  // 通知設置
  notifications: {
    lowBattery: true,
    maxDepthReached: true,
    maxDistanceReached: true,
    connectionLost: true,
    obstacleDetected: true,
  },

  // AUV 機型配置
  auvModels: {
    small: {
      name: "小型 AUV",
      maxDepth: 20,
      maxDistance: 200,
      maxSpeed: 1.5,
      battery: 4,
      description: "適合淺海勘測，最大下潛深度 20 米",
    },
    medium: {
      name: "中型 AUV",
      maxDepth: 100,
      maxDistance: 500,
      maxSpeed: 2.0,
      battery: 8,
      description: "適合海洋調查，最大下潛深度 100 米",
    },
    large: {
      name: "大型 AUV",
      maxDepth: 500,
      maxDistance: 2000,
      maxSpeed: 2.5,
      battery: 12,
      description: "適合深海探測，最大下潛深度 500 米",
    },
    deep: {
      name: "深海 AUV",
      maxDepth: 6000,
      maxDistance: 5000,
      maxSpeed: 1.5,
      battery: 16,
      description: "適合超深海作業，最大下潛深度 6000 米",
    },
  },

  // 預設選擇
  defaultModel: "medium" as const,
};
