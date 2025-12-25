import React, { useState, useEffect, useRef } from "react";
import { Button } from "@/components/ui/button";
import { Card } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Input } from "@/components/ui/input";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Upload, X, Save, Plus, GripVertical, Trash2 } from "lucide-react";
import { toast } from "sonner";
import { useDemoMode } from "@/context/DemoModeContext";
import { demoMissionSetupData, demoMissionManagerData } from "@/data/demoData";
import { useSearchParams } from "react-router-dom";

// --- Block types (module scope) ---
// Block AST types
type BlockAction = {
  id: string;
  type: 'action';
  name: string;
  params?: Record<string, any>;
};
type BlockIf = {
  id: string;
  type: 'if';
  condition?: string;
  then: Block[];
  else: Block[];
};
type BlockRepeat = {
  id: string;
  type: 'repeat';
  times: number;
  children: Block[];
};
type BlockForever = {
  id: string;
  type: 'forever';
  children: Block[];
};
type BlockVariable = {
  id: string;
  type: 'variable';
  variable: string;
  value?: string | number;
};
type BlockList = {
  id: string;
  type: 'list';
  listName?: string;
  items: any[];
};
type Block = BlockAction | BlockIf | BlockRepeat | BlockForever | BlockVariable | BlockList;

// MissionSetup component
const MissionSetup: React.FC = () => {
  
  const demoMode = useDemoMode();
  const [searchParams] = useSearchParams();

  const [images, setImages] = useState<Array<string | null>>([null, null, null, null, null]);
  const [mode, setMode] = useState<'simple' | 'advanced'>('simple');

  const [bboxes, setBboxes] = useState<Array<{ x: number; y: number; width: number; height: number } | null>>([
    null,
    null,
    null,
    null,
    null,
  ]);

  const [confidence, setConfidence] = useState([75]);
  const [targetType, setTargetType] = useState("");
  const [customTarget, setCustomTarget] = useState("");
  const [triggerType, setTriggerType] = useState("");
  const [triggerValue, setTriggerValue] = useState("");

  const [actions, setActions] = useState<Array<{ id: string; name: string }>>([]);
  const [draggedAction, setDraggedAction] = useState<string | null>(null);

  const [canvasOpen, setCanvasOpen] = useState(false);
  const [canvasImageIndex, setCanvasImageIndex] = useState<number | null>(null);

  const [blocks, setBlocks] = useState<Block[]>([]);

  // 拖曳排序狀態
  const [dragBlockId, setDragBlockId] = useState<string | null>(null);
  const [dragOverBlockId, setDragOverBlockId] = useState<string | null>(null);
  const [dragOverGapIndex, setDragOverGapIndex] = useState<number | null>(null);
  const [dragOverBranch, setDragOverBranch] = useState<{ targetId: string; index: number } | null>(null);
  const [selectedBlockId, setSelectedBlockId] = useState<string | null>(null);
  const isDraggingRef = useRef(false);
  const [undoStack, setUndoStack] = useState<Block[][]>([]);
  const [redoStack, setRedoStack] = useState<Block[][]>([]);
  const applyingHistoryRef = React.useRef(false);
  const prevBlocksRef = React.useRef<Block[] | null>(null);

  // ensure prevBlocksRef is initialized once (avoid useEffect linter issues)
  if (prevBlocksRef.current === null) prevBlocksRef.current = blocks;

  // record history on blocks change
  useEffect(() => {
    if (applyingHistoryRef.current) return;
    const prev = prevBlocksRef.current;
    if (prev) {
      setUndoStack((s) => [...s, prev]);
      setRedoStack([]);
    }
    prevBlocksRef.current = blocks;
  }, [blocks]);

  const undo = React.useCallback(() => {
    setUndoStack((u) => {
      if (u.length === 0) return u;
      const last = u[u.length - 1];
      applyingHistoryRef.current = true;
      setRedoStack((r) => [blocks, ...r]);
      setBlocks(last);
      applyingHistoryRef.current = false;
      return u.slice(0, -1);
    });
  }, [blocks]);

  const redo = React.useCallback(() => {
    setRedoStack((r) => {
      if (r.length === 0) return r;
      const [first, ...rest] = r;
      applyingHistoryRef.current = true;
      setUndoStack((u) => [...u, blocks]);
      setBlocks(first);
      applyingHistoryRef.current = false;
      return rest;
    });
  }, [blocks]);

  const deleteBlock = React.useCallback((id: string) => {
    setBlocks((prev) => removeBlockById(prev, id));
    setSelectedBlockId((sel) => (sel === id ? null : sel));
    toast.success("已刪除積木");
  }, []);

  // keyboard handlers: Delete, Ctrl+Z, Ctrl+Y
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Delete' && selectedBlockId) {
        deleteBlock(selectedBlockId);
      }
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') {
        e.preventDefault();
        undo();
      }
      if ((e.ctrlKey || e.metaKey) && (e.key.toLowerCase() === 'y' || (e.shiftKey && e.key.toLowerCase() === 'z'))) {
        e.preventDefault();
        redo();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [selectedBlockId, blocks, deleteBlock, undo, redo]);

  const actionLibrary = demoMode ? demoMissionSetupData.actionLibrary.map((a) => a.name) : [];

  useEffect(() => {
    if (!demoMode) return;
    const id = searchParams.get("id");
    if (!id) return;
    const seqs = demoMissionSetupData.actionSequences;
    const seq = seqs[(Number(id) - 1) % Math.max(1, seqs.length)];
    if (seq) setActions(seq.actions.map((a) => ({ id: a.id, name: a.name })));
    if (demoMissionSetupData.targetSettings) {
      setTargetType(demoMissionSetupData.targetSettings.objectType || "");
      setConfidence([demoMissionSetupData.targetSettings.detectionThreshold || 75]);
    }
  }, [demoMode, searchParams]);

  

  // Global cleanup for drag events (in case dragend not fired on element)
  useEffect(() => {
    const onUp = () => {
      isDraggingRef.current = false;
      setDragBlockId(null);
      setDragOverBlockId(null);
      setDragOverGapIndex(null);
      cleanupDragPreview();
    };
    window.addEventListener('mouseup', onUp);
    window.addEventListener('dragend', onUp);
    return () => {
      window.removeEventListener('mouseup', onUp);
      window.removeEventListener('dragend', onUp);
    };
  }, []);

  if (!demoMode) {
    return (
      <div className="min-h-screen p-6 space-y-6 flex items-center justify-center">
        <Card className="glass p-12 text-center max-w-md">
          <h1 className="text-2xl font-bold mb-4">任務設置</h1>
          <p className="text-muted-foreground mb-6">展示模式已關閉。請在系統設置中啟用展示模式或連接真實設備。</p>
          <p className="text-sm text-muted-foreground">真實 API 連接功能關閉中...</p>
        </Card>
      </div>
    );
  }

  const openCanvasEditor = (index: number) => {
    setCanvasImageIndex(index);
    setCanvasOpen(true);
  };
  const closeCanvasEditor = () => {
    setCanvasOpen(false);
    setCanvasImageIndex(null);
  };

  const handleImageUpload = (index: number, file: File) => {
    const reader = new FileReader();
    reader.onload = (e) => {
      const result = e.target?.result as string;
      if (!result || !result.startsWith("data:image")) {
        toast.error("圖片格式錯誤，請重新上傳");
        return;
      }
      const newImages = [...images];
      newImages[index] = result;
      setImages(newImages);
      setBboxes((prev) => {
        const updated = [...prev];
        updated[index] = { x: 20, y: 20, width: 60, height: 60 };
        return updated;
      });
      toast.success(`照片 ${index + 1} 已上傳`);
    };
    reader.readAsDataURL(file);
  };

  const handleRemoveImage = (index: number) => {
    const newImages = [...images];
    newImages[index] = null;
    setImages(newImages);
    setBboxes((prev) => {
      const updated = [...prev];
      updated[index] = null;
      return updated;
    });
  };

  const handleDragStart = (e: React.DragEvent, actionName: string) => {
    e.dataTransfer.effectAllowed = "copy";
    setDraggedAction(actionName);
  };
  const handlePaletteDragEnd = (e: React.DragEvent) => {
    isDraggingRef.current = false;
    try {
      e.dataTransfer.clearData('application/x-block');
    } catch (err) {
      // ignore
    }
    setDragOverGapIndex(null);
    cleanupDragPreview();
  };
  const cleanupDragPreview = () => {
    try {
      const p = (isDraggingRef as unknown as { currentPreview?: HTMLElement }).currentPreview;
      if (p && p.parentNode) p.parentNode.removeChild(p);
    } catch (err) {
      // ignore
    }
    (isDraggingRef as unknown as { currentPreview?: HTMLElement }).currentPreview = undefined;
  };
  const handlePaletteMouseUp = () => {
    // Some platforms don't reliably fire dragend; ensure we clear dragging state on mouseup
    isDraggingRef.current = false;
    setDragOverGapIndex(null);
  };

  const paletteClickGuard = (fn: () => void) => (e: React.MouseEvent) => {
    if (isDraggingRef.current) {
      // ignore clicks that are part of a drag
      isDraggingRef.current = false;
      return;
    }
    fn();
  };
  // palette drag start (for blocks)
  const handlePaletteDragStart = (e: React.DragEvent, type: string, payload?: unknown) => {
    e.dataTransfer.effectAllowed = 'copy';
    const data = JSON.stringify({ type, payload });
    e.dataTransfer.setData('application/x-block', data);
    isDraggingRef.current = true;
    // set drag preview image
    createAndSetDragImage(e, type.toUpperCase(), '#0284c7');
  };
  // create a simple drag preview element and set it as drag image
  const createAndSetDragImage = (e: React.DragEvent, label: string, color = '#0ea5e9') => {
    try {
      const preview = document.createElement('div');
      preview.style.padding = '8px 12px';
      preview.style.background = color;
      preview.style.color = '#fff';
      preview.style.fontWeight = '600';
      preview.style.borderRadius = '8px';
      preview.style.boxShadow = '0 4px 12px rgba(0,0,0,0.15)';
      preview.style.position = 'absolute';
      preview.style.top = '-9999px';
      preview.style.left = '-9999px';
      preview.innerText = label;
      document.body.appendChild(preview);
      // offset so cursor sits near top-left of preview
      e.dataTransfer.setDragImage(preview, 16, 16);
      // store on ref for cleanup
      (isDraggingRef as unknown as { currentPreview?: HTMLElement }).currentPreview = preview;
    } catch (err) {
      // ignore
    }
  };
  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.dataTransfer.dropEffect = "copy";
  };
  const handleDrop = (e: React.DragEvent) => {
    e.preventDefault();
    if (draggedAction) {
      const newAction = { id: `${draggedAction}-${Date.now()}`, name: draggedAction };
      setActions((prev) => [...prev, newAction]);
      toast.success(`已添加動作: ${draggedAction}`);
      setDraggedAction(null);
    }
  };

  const handleAddAction = (actionName: string) => {
    const newAction = { id: `${actionName}-${Date.now()}`, name: actionName };
    setActions((prev) => [...prev, newAction]);
    toast.success(`已添加動作: ${actionName}`);
  };

  const handleRemoveAction = (id: string) => {
    const removedAction = actions.find((a) => a.id === id)?.name;
    setActions((prev) => prev.filter((a) => a.id !== id));
    toast.success(`已移除動作: ${removedAction}`);
  };

  // ...existing code...

  // 新增積木
  // 新增積木 (並選取以便快速編輯)
  const addBlock = (block: Block) => {
    setBlocks((prev) => {
      const next = [...prev, block];
      return next;
    });
    // select newly added block so inspector / inline inputs show
    setTimeout(() => setSelectedBlockId(block.id), 0);
  };

  // helpers to create blocks
  const createActionBlock = (name = "動作") => ({ id: `action-${Date.now()}-${Math.random().toString(16).slice(2,6)}`, type: 'action' as const, name });
  const createIfBlock = (condition = "condition") => ({ id: `if-${Date.now()}-${Math.random().toString(16).slice(2,6)}`, type: 'if' as const, condition, then: [] as Block[], else: [] as Block[] });
  const createRepeatBlock = (times = 3) => ({ id: `repeat-${Date.now()}-${Math.random().toString(16).slice(2,6)}`, type: 'repeat' as const, times, children: [] as Block[] });
  const createForeverBlock = () => ({ id: `forever-${Date.now()}-${Math.random().toString(16).slice(2,6)}`, type: 'forever' as const, children: [] as Block[] });
  const createVariableBlock = (variable = 'var', value: string | number = 0) => ({ id: `var-${Date.now()}-${Math.random().toString(16).slice(2,6)}`, type: 'variable' as const, variable, value });
  const createListBlock = (listName = 'list') => ({ id: `list-${Date.now()}-${Math.random().toString(16).slice(2,6)}`, type: 'list' as const, listName, items: [] as (string|number)[] });

  // 更新積木（遞迴）
  function updateBlockById(bs: Block[], id: string, patch: Partial<Block>): Block[] {
    return bs.map((b) => {
      if (b.id === id) return { ...b, ...(patch as Partial<Block>) } as Block;
      if (b.type === "if") return { ...b, then: updateBlockById(b.then, id, patch), else: updateBlockById(b.else, id, patch) };
      if (b.type === "repeat") return { ...b, children: updateBlockById(b.children, id, patch) };
      return b;
    });
  }

  

  // 排序函式
  function moveBlock(blocks: Block[], fromId: string, toId: string): Block[] {
    const fromIdx = blocks.findIndex((b) => b.id === fromId);
    const toIdx = blocks.findIndex((b) => b.id === toId);
    if (fromIdx === -1 || toIdx === -1 || fromIdx === toIdx) return blocks;
    const arr = [...blocks];
    const [moved] = arr.splice(fromIdx, 1);
    arr.splice(toIdx, 0, moved);
    return arr;
  }

  // UI: 積木渲染
  const renderBlock = (block: Block, depth = 0, parentBlocks = blocks, setParentBlocks = setBlocks) => {
    const handleDragStart = (e: React.DragEvent) => {
      setDragBlockId(block.id);
      e.dataTransfer.effectAllowed = "move";
      isDraggingRef.current = true;
      // set preview for existing block drag
      createAndSetDragImage(e, block.type.toUpperCase(), block.type === 'action' ? '#E67E22' : '#06b6d4');
    };
    const handleDragEndInner = (e: React.DragEvent) => {
      isDraggingRef.current = false;
      setDragBlockId(null);
      setDragOverBlockId(null);
      setDragOverGapIndex(null);
      cleanupDragPreview();
    };
    const handleClickSelect = (e: React.MouseEvent) => {
      // avoid selecting while dragging
      if (isDraggingRef.current) return;
      setSelectedBlockId(selectedBlockId === block.id ? null : block.id);
    };
    const handleDragOver = (e: React.DragEvent) => {
      e.preventDefault();
      setDragOverBlockId(block.id);
    };
    const handleDrop = (e: React.DragEvent) => {
      e.preventDefault();
      if (dragBlockId && dragBlockId !== block.id) {
        setParentBlocks(moveBlock(parentBlocks, dragBlockId, block.id));
      }
      setDragBlockId(null);
      setDragOverBlockId(null);
    };

    const handleDragEnd = (e: React.DragEvent) => {
      handleDragEndInner(e);
    };
    const highlight = dragOverBlockId === block.id ? "ring-4 ring-cyan-400 shadow-lg" : "";

    // 嵌套 drop 區
    const handleDropToBranch = (branch: "then" | "else" | "children") => (e: React.DragEvent) => {
      e.preventDefault();
      e.stopPropagation();
      // if dragging from palette, create new block
      const paletteData = e.dataTransfer.getData('application/x-block');
      if (paletteData) {
        try {
          const parsed = JSON.parse(paletteData);
          let newB: Block | null = null;
          if (parsed.type === 'action') newB = createActionBlock(parsed.payload?.name || '動作');
          if (parsed.type === 'if') newB = createIfBlock(parsed.payload?.condition || 'condition');
          if (parsed.type === 'repeat') newB = createRepeatBlock(parsed.payload?.times || 3);
          if (parsed.type === 'forever') newB = createForeverBlock();
          if (parsed.type === 'variable') newB = createVariableBlock(parsed.payload?.variable || 'var', parsed.payload?.value ?? 0);
          if (parsed.type === 'list') newB = createListBlock(parsed.payload?.listName || 'list');
          if (newB) {
            setBlocks((prev) => insertBlock(prev, block.id, newB, branch));
            toast.success('已從積木庫新增積木');
          }
        } catch (err) {
          console.warn('Invalid palette drop data', err);
        }
        setDragOverBlockId(null);
        return;
      }
      if (!dragBlockId) return;
      // 找到拖曳積木
      function findBlock(bs: Block[]): Block | undefined {
        for (const b of bs) {
          if (b.id === dragBlockId) return b;
          if (b.type === "if") {
            const t = findBlock(b.then);
            if (t) return t;
            const e = findBlock(b.else);
            if (e) return e;
          }
          if (b.type === "repeat") {
            const c = findBlock(b.children);
            if (c) return c;
          }
        }
        return undefined;
      }
      const dragged = findBlock(blocks);
      if (!dragged) return;
      // 先移除原位置
      const removed = removeBlockById(blocks, dragBlockId);
      // 插入到目標分支
      setBlocks(insertBlock(removed, block.id, dragged, branch));
      setDragBlockId(null);
      setDragOverBlockId(null);
    };

    const handleDropToBranchAt = (branch: "then" | "else" | "children", index: number) => (e: React.DragEvent) => {
      e.preventDefault();
      e.stopPropagation();
      const paletteData = e.dataTransfer.getData('application/x-block');
      if (paletteData) {
        try {
          const parsed = JSON.parse(paletteData);
          let newB: Block | null = null;
          if (parsed.type === 'action') newB = createActionBlock(parsed.payload?.name || '動作');
          if (parsed.type === 'if') newB = createIfBlock(parsed.payload?.condition || 'condition');
          if (parsed.type === 'repeat') newB = createRepeatBlock(parsed.payload?.times || 3);
          if (parsed.type === 'forever') newB = createForeverBlock();
          if (parsed.type === 'variable') newB = createVariableBlock(parsed.payload?.variable || 'var', parsed.payload?.value ?? 0);
          if (parsed.type === 'list') newB = createListBlock(parsed.payload?.listName || 'list');
          if (newB) {
            setBlocks((prev) => insertBlockAt(prev, block.id, newB as Block, branch, index));
            toast.success('已從積木庫新增積木');
          }
        } catch (err) {
          console.warn('Invalid palette drop data', err);
        }
        setDragOverBranch(null);
        return;
      }
      if (!dragBlockId) return;
      if (dragBlockId === block.id) return;
      // prevent dropping a node into its own descendant
      if (isDescendant(blocks, dragBlockId, block.id)) {
        toast.error('無法將節點放入其子節點');
        setDragOverBranch(null);
        return;
      }
      // find original info
      const origInfo = findParentAndIndex(blocks, dragBlockId);
      const origIndex = origInfo ? origInfo.index : -1;
      const origParentId = findParentBlockId(blocks, dragBlockId);
      const removed = removeBlockById(blocks, dragBlockId);
      // adjust insertion index if moving within same parent and original index < target index
      let adjusted = index;
      if (origParentId === block.id && origIndex !== -1 && origIndex < index) adjusted = Math.max(0, index - 1);
      // find dragged node (from pre-removal state)
      function findBlockNode(bs: Block[]): Block | undefined {
        for (const b of bs) {
          if (b.id === dragBlockId) return b;
          if (b.type === 'if') {
            const t = findBlockNode(b.then);
            if (t) return t;
            const e = findBlockNode(b.else);
            if (e) return e;
          }
          if (b.type === 'repeat' || b.type === 'forever') {
            const c = findBlockNode((b as BlockRepeat | BlockForever).children);
            if (c) return c;
          }
        }
        return undefined;
      }
      const dragged = findBlockNode(blocks);
      if (!dragged) return;
      setBlocks(insertBlockAt(removed, block.id, dragged, branch, adjusted));
      setDragBlockId(null);
      setDragOverBranch(null);
    };

    if (block.type === "action") {
      return (
          <div
          key={block.id}
          draggable
            onDragStart={handleDragStart}
            onDragEnd={handleDragEnd}
          onDragOver={handleDragOver}
          onDrop={handleDrop}
            onClick={handleClickSelect}
          style={{ marginLeft: depth * 24, boxShadow: '0 2px 8px rgba(0,0,0,0.10)', borderRadius: 12, background: '#FFB366', border: '2px solid #E67E22', clipPath: 'polygon(0 0, 20px 0, 28px 8px, calc(100% - 28px) 8px, calc(100% - 20px) 0, 100% 0, 100% 100%, calc(100% - 20px) 100%, calc(100% - 28px) calc(100% - 8px), 28px calc(100% - 8px), 20px 100%, 0 100%)' }}
          className={`px-4 py-2 mb-2 flex items-center cursor-move ${highlight}`}
        >
          <div className="flex-1 flex items-center">
            <span className="font-bold mr-2" style={{ color: '#A04000' }}>動作：</span>
            {selectedBlockId === block.id ? (
              <Input value={block.name} onChange={(e) => setBlocks(updateBlockById(blocks, block.id, { name: e.target.value }))} className="h-7 text-sm bg-[#FFECB3] rounded-none border-none shadow-none px-2" style={{ background: '#FFECB3', color: '#A04000' }} />
            ) : (
              <span className="text-base" style={{ color: '#A04000' }}>{block.name}</span>
            )}
          </div>
            <div className="flex items-center gap-2 ml-2">
            <Button size="sm" variant="outline" onClick={() => setSelectedBlockId(selectedBlockId === block.id ? null : block.id)}>
              {selectedBlockId === block.id ? "Done" : "Edit"}
            </Button>
            <Button size="sm" variant="ghost" onClick={() => deleteBlock(block.id)}>
              <Trash2 className="w-4 h-4 text-destructive" />
            </Button>
          </div>
        </div>
      );
    }
    if (block.type === "if") {
      return (
        <div
          key={block.id}
          draggable
          onDragStart={handleDragStart}
          onDragOver={handleDragOver}
          onDrop={handleDrop}
          style={{ marginLeft: depth * 24, boxShadow: '0 2px 8px rgba(0,0,0,0.10)', borderRadius: 12, background: '#F7DC6F', border: '2px solid #B7950B', clipPath: 'polygon(0 0, 20px 0, 28px 8px, calc(100% - 28px) 8px, calc(100% - 20px) 0, 100% 0, 100% 100%, calc(100% - 20px) 100%, calc(100% - 28px) calc(100% - 8px), 28px calc(100% - 8px), 20px 100%, 0 100%)' }}
          className={`px-4 py-2 mb-2 cursor-move ${highlight}`}
        >
          <div className="flex items-center justify-between mb-1">
            <div className="font-bold" style={{ color: '#B7950B' }}>如果：{selectedBlockId === block.id ? (
              <Input value={block.condition} onChange={(e) => setParentBlocks(updateBlockById(parentBlocks, block.id, { ...(block as BlockIf), condition: e.target.value }))} className="h-7 text-sm bg-[#FFF9E3] rounded-none border-none shadow-none px-2" style={{ background: '#FFF9E3', color: '#B7950B' }} />
            ) : (
              <span className="text-base" style={{ color: '#B7950B' }}>{block.condition}</span>
            )}</div>
            <div className="flex items-center gap-2">
              <Button size="sm" variant="outline" onClick={() => setSelectedBlockId(selectedBlockId === block.id ? null : block.id)}>
                {selectedBlockId === block.id ? "Done" : "Edit"}
              </Button>
              <Button size="sm" variant="ghost" onClick={() => deleteBlock(block.id)}>
                <Trash2 className="w-4 h-4 text-destructive" />
              </Button>
            </div>
          </div>
            <div className="ml-4">
            <div className="font-semibold mb-1" style={{ color: '#B7950B' }}>Then：</div>
            <div
              style={{ minHeight: 32, background: '#FCF3CF', borderRadius: 8, border: '2px dashed #F7DC6F', marginBottom: 4, padding: 8, boxShadow: dragOverBlockId === block.id ? '0 0 0 8px rgba(6,182,212,0.25)' : undefined, transform: dragOverBlockId === block.id ? 'translateY(4px)' : undefined, transition: 'all 150ms ease' }}
              onDragOver={(e) => { e.preventDefault(); setDragOverBlockId(block.id); e.dataTransfer.dropEffect = 'copy'; }}
              onDrop={handleDropToBranch("then")}
            >
              {block.then.map((b, i) => (
                <React.Fragment key={b.id}>
                  <div
                    className={`h-2 ${dragOverBranch?.targetId === block.id && dragOverBranch.index === i ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'}`}
                    onDragOver={(e) => { e.preventDefault(); setDragOverBranch({ targetId: block.id, index: i }); e.dataTransfer.dropEffect = 'move'; }}
                    onDrop={handleDropToBranchAt('then', i)}
                  />
                  {renderBlock(b, depth + 1, block.then, (newThen) => {
                    (setParentBlocks as unknown as (b: Block[]) => void)(parentBlocks.map((bl) => (bl.id === block.id ? { ...block, then: newThen as Block[] } : bl)));
                  })}
                </React.Fragment>
              ))}
              <div
                className={`h-2 ${dragOverBranch?.targetId === block.id && dragOverBranch.index === block.then.length ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'}`}
                onDragOver={(e) => { e.preventDefault(); setDragOverBranch({ targetId: block.id, index: block.then.length }); e.dataTransfer.dropEffect = 'move'; }}
                onDrop={handleDropToBranchAt('then', block.then.length)}
              />
                {selectedBlockId === block.id && (
                  <div className="mt-2 flex gap-2">
                    <Select value={""} onValueChange={(v) => setBlocks(updateBlockById(blocks, block.id, { then: [...block.then, { id: `a-${Date.now()}`, type: 'action', name: v }] }))}>
                      <SelectTrigger className="bg-[#FFF9E3] border-none shadow-none rounded-none px-2 text-[#B7950B]">
                        <SelectValue placeholder="從庫中新增動作到 Then" />
                      </SelectTrigger>
                      <SelectContent className="bg-[#FFF9E3] border-none shadow-none rounded-none text-[#B7950B]">
                        {actionLibrary.map((name) => (
                          <SelectItem key={name} value={name}>{name}</SelectItem>
                        ))}
                      </SelectContent>
                    </Select>
                  </div>
                )}
            </div>
          </div>
            <div className="ml-4">
            <div className="font-semibold mb-1" style={{ color: '#B7950B' }}>Else：</div>
            <div
              style={{ minHeight: 32, background: '#FCF3CF', borderRadius: 8, border: '2px dashed #F7DC6F', marginBottom: 4, padding: 8, boxShadow: dragOverBlockId === block.id ? '0 0 0 8px rgba(6,182,212,0.25)' : undefined, transform: dragOverBlockId === block.id ? 'translateY(4px)' : undefined, transition: 'all 150ms ease' }}
              onDragOver={(e) => { e.preventDefault(); setDragOverBlockId(block.id); e.dataTransfer.dropEffect = 'copy'; }}
              onDrop={handleDropToBranch("else")}
            >
              {block.else.map((b, i) => (
                <React.Fragment key={b.id}>
                  <div
                    className={`h-2 ${dragOverBranch?.targetId === block.id && dragOverBranch.index === i ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'}`}
                    onDragOver={(e) => { e.preventDefault(); setDragOverBranch({ targetId: block.id, index: i }); e.dataTransfer.dropEffect = 'move'; }}
                    onDrop={handleDropToBranchAt('else', i)}
                  />
                  {renderBlock(b, depth + 1, block.else, (newElse) => {
                    (setParentBlocks as unknown as (b: Block[]) => void)(parentBlocks.map((bl) => (bl.id === block.id ? { ...block, else: newElse as Block[] } : bl)));
                  })}
                </React.Fragment>
              ))}
              <div
                className={`h-2 ${dragOverBranch?.targetId === block.id && dragOverBranch.index === block.else.length ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'}`}
                onDragOver={(e) => { e.preventDefault(); setDragOverBranch({ targetId: block.id, index: block.else.length }); e.dataTransfer.dropEffect = 'move'; }}
                onDrop={handleDropToBranchAt('else', block.else.length)}
              />
              {selectedBlockId === block.id && (
                <div className="mt-2 flex gap-2">
                  <Select value={""} onValueChange={(v) => setBlocks(updateBlockById(blocks, block.id, { else: [...block.else, { id: `a-${Date.now()}`, type: 'action', name: v }] }))}>
                    <SelectTrigger className="bg-[#FFF9E3] border-none shadow-none rounded-none px-2 text-[#B7950B]">
                      <SelectValue placeholder="從庫中新增動作到 Else" />
                    </SelectTrigger>
                    <SelectContent className="bg-[#FFF9E3] border-none shadow-none rounded-none text-[#B7950B]">
                      {actionLibrary.map((name) => (
                        <SelectItem key={name} value={name}>{name}</SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                </div>
              )}
            </div>
          </div>
        </div>
      );
    }
    if (block.type === "repeat") {
      return (
        <div
          key={block.id}
          draggable
          onDragStart={handleDragStart}
          onDragOver={handleDragOver}
          onDrop={handleDrop}
          style={{ marginLeft: depth * 24, boxShadow: '0 2px 8px rgba(0,0,0,0.10)', borderRadius: 12, background: '#82E0AA', border: '2px solid #229954', clipPath: 'polygon(0 0, 20px 0, 28px 8px, calc(100% - 28px) 8px, calc(100% - 20px) 0, 100% 0, 100% 100%, calc(100% - 20px) 100%, calc(100% - 28px) calc(100% - 8px), 28px calc(100% - 8px), 20px 100%, 0 100%)' }}
          className={`px-4 py-2 mb-2 cursor-move ${highlight}`}
        >
          <div className="flex items-center justify-between mb-1">
            <div className="font-bold" style={{ color: '#229954' }}>重複 {selectedBlockId === block.id ? (
              <Input type="number" value={String(block.times)} onChange={(e) => setParentBlocks(updateBlockById(parentBlocks, block.id, { ...(block as BlockRepeat), times: Number(e.target.value) }))} className="h-7 text-sm bg-[#E8F8F5] rounded-none border-none shadow-none px-2 w-16" style={{ background: '#E8F8F5', color: '#229954' }} />
            ) : (
              <span className="text-base" style={{ color: '#229954' }}>{block.times}</span>
            )} 次</div>
            <div className="flex items-center gap-2">
              <Button size="sm" variant="outline" onClick={() => setSelectedBlockId(selectedBlockId === block.id ? null : block.id)}>
                {selectedBlockId === block.id ? "Done" : "Edit"}
              </Button>
              <Button size="sm" variant="ghost" onClick={() => deleteBlock(block.id)}>
                <Trash2 className="w-4 h-4 text-destructive" />
              </Button>
            </div>
          </div>
          <div
            style={{ minHeight: 32, background: '#D5F5E3', borderRadius: 8, border: '2px dashed #82E0AA', marginBottom: 4, padding: 8, boxShadow: dragOverBlockId === block.id ? '0 0 0 8px rgba(6,182,212,0.25)' : undefined, transform: dragOverBlockId === block.id ? 'translateY(4px)' : undefined, transition: 'all 150ms ease' }}
            onDragOver={(e) => { e.preventDefault(); setDragOverBlockId(block.id); e.dataTransfer.dropEffect = 'copy'; }}
            onDrop={handleDropToBranch("children")}
          >
              {block.children.map((b, i) => (
                <React.Fragment key={b.id}>
                  <div
                    className={`h-2 ${dragOverBranch?.targetId === block.id && dragOverBranch.index === i ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'}`}
                    onDragOver={(e) => { e.preventDefault(); setDragOverBranch({ targetId: block.id, index: i }); e.dataTransfer.dropEffect = 'move'; }}
                    onDrop={handleDropToBranchAt('children', i)}
                  />
                  {renderBlock(b, depth + 1, block.children, (newChildren) => {
                    (setParentBlocks as unknown as (b: Block[]) => void)(parentBlocks.map((bl) => (bl.id === block.id ? { ...block, children: newChildren as Block[] } : bl)));
                  })}
                </React.Fragment>
              ))}
              <div
                className={`h-2 ${dragOverBranch?.targetId === block.id && dragOverBranch.index === block.children.length ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'}`}
                onDragOver={(e) => { e.preventDefault(); setDragOverBranch({ targetId: block.id, index: block.children.length }); e.dataTransfer.dropEffect = 'move'; }}
                onDrop={handleDropToBranchAt('children', block.children.length)}
              />
            {selectedBlockId === block.id && (
              <div className="mt-2">
                <Select value={""} onValueChange={(v) => setBlocks(updateBlockById(blocks, block.id, { children: [...block.children, { id: `a-${Date.now()}`, type: 'action', name: v }] }))}>
                  <SelectTrigger className="bg-[#E8F8F5] border-none shadow-none rounded-none px-2 text-[#229954]">
                    <SelectValue placeholder="從庫中新增動作到 Repeat" />
                  </SelectTrigger>
                  <SelectContent className="bg-[#E8F8F5] border-none shadow-none rounded-none text-[#229954]">
                    {actionLibrary.map((name) => (
                      <SelectItem key={name} value={name}>{name}</SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </div>
            )}
          </div>
        </div>
      );
    }
    if (block.type === 'forever') {
      return (
        <div key={block.id} draggable onDragStart={handleDragStart} onDragOver={handleDragOver} onDrop={handleDrop} onClick={handleClickSelect} style={{ marginLeft: depth * 24, boxShadow: '0 2px 8px rgba(0,0,0,0.10)', borderRadius: 12, background: '#A0D8FF', border: '2px solid #0ea5e9', clipPath: 'polygon(0 0, 20px 0, 28px 8px, calc(100% - 28px) 8px, calc(100% - 20px) 0, 100% 0, 100% 100%, calc(100% - 20px) 100%, calc(100% - 28px) calc(100% - 8px), 28px calc(100% - 8px), 20px 100%, 0 100%)' }} className={`px-4 py-2 mb-2 cursor-move ${highlight}`}>
          <div className="flex items-center justify-between mb-1">
            <div className="font-bold" style={{ color: '#0b5f88' }}>永久循環</div>
            <div className="flex items-center gap-2">
              <Button size="sm" variant="outline" onClick={() => setSelectedBlockId(selectedBlockId === block.id ? null : block.id)}>{selectedBlockId === block.id ? 'Done' : 'Edit'}</Button>
              <Button size="sm" variant="ghost" onClick={() => deleteBlock(block.id)}><Trash2 className="w-4 h-4 text-destructive" /></Button>
            </div>
          </div>
            <div style={{ minHeight: 32, background: '#E6F7FF', borderRadius: 8, border: '2px dashed #A0D8FF', marginBottom: 4, padding: 8 }} onDragOver={(e) => { e.preventDefault(); setDragOverBlockId(block.id); e.dataTransfer.dropEffect = 'copy'; }} onDrop={handleDropToBranch('children')}>
              {block.children.map((b, i) => (
                <React.Fragment key={b.id}>
                  <div
                    className={`h-2 ${dragOverBranch?.targetId === block.id && dragOverBranch.index === i ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'}`}
                    onDragOver={(e) => { e.preventDefault(); setDragOverBranch({ targetId: block.id, index: i }); e.dataTransfer.dropEffect = 'move'; }}
                    onDrop={handleDropToBranchAt('children', i)}
                  />
                  {renderBlock(b, depth + 1, block.children, (newChildren) => { (setParentBlocks as unknown as (b: Block[]) => void)(parentBlocks.map((bl) => (bl.id === block.id ? { ...block, children: newChildren as Block[] } : bl))); })}
                </React.Fragment>
              ))}
              <div
                className={`h-2 ${dragOverBranch?.targetId === block.id && dragOverBranch.index === block.children.length ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'}`}
                onDragOver={(e) => { e.preventDefault(); setDragOverBranch({ targetId: block.id, index: block.children.length }); e.dataTransfer.dropEffect = 'move'; }}
                onDrop={handleDropToBranchAt('children', block.children.length)}
              />
            {selectedBlockId === block.id && (
              <div className="mt-2">
                <Select value={""} onValueChange={(v) => setBlocks(updateBlockById(blocks, block.id, { children: [...block.children, { id: `a-${Date.now()}`, type: 'action', name: v }] }))}>
                  <SelectTrigger className="bg-[#E6F7FF] border-none shadow-none rounded-none px-2 text-[#0b5f88]"><SelectValue placeholder="從庫中新增動作到 Forever" /></SelectTrigger>
                  <SelectContent className="bg-[#E6F7FF] border-none shadow-none rounded-none text-[#0b5f88]">{actionLibrary.map((name) => (<SelectItem key={name} value={name}>{name}</SelectItem>))}</SelectContent>
                </Select>
              </div>
            )}
          </div>
        </div>
      );
    }
    if (block.type === 'variable') {
      return (
        <div key={block.id} draggable onDragStart={handleDragStart} onDragOver={handleDragOver} onDrop={handleDrop} onClick={handleClickSelect} style={{ marginLeft: depth * 24, boxShadow: '0 2px 8px rgba(0,0,0,0.10)', borderRadius: 12, background: '#FFD6A5', border: '2px solid #FB923C' }} className={`px-4 py-2 mb-2 cursor-move ${highlight}`}>
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-3">
              <span className="font-bold" style={{ color: '#9a4b00' }}>變數：</span>
              {selectedBlockId === block.id ? (
                <Input value={(block as BlockVariable).variable} onChange={(e) => setParentBlocks(updateBlockById(parentBlocks, block.id, { ...(block as BlockVariable), variable: e.target.value }))} className="h-7 text-sm bg-[#FFF4E6] rounded-none border-none shadow-none px-2" />
              ) : (
                <span style={{ color: '#9a4b00' }}>{(block as BlockVariable).variable}</span>
              )}
              <span className="ml-2 text-sm text-muted-foreground">=</span>
              {selectedBlockId === block.id ? (
                <Input value={String((block as BlockVariable).value)} onChange={(e) => setParentBlocks(updateBlockById(parentBlocks, block.id, { ...(block as BlockVariable), value: isNaN(Number(e.target.value)) ? e.target.value : Number(e.target.value) }))} className="h-7 text-sm bg-[#FFF4E6] rounded-none border-none shadow-none px-2 w-24" />
              ) : (
                <span style={{ color: '#9a4b00' }}>{String((block as BlockVariable).value)}</span>
              )}
            </div>
            <div className="flex items-center gap-2">
              <Button size="sm" variant="outline" onClick={() => setSelectedBlockId(selectedBlockId === block.id ? null : block.id)}>{selectedBlockId === block.id ? 'Done' : 'Edit'}</Button>
              <Button size="sm" variant="ghost" onClick={() => deleteBlock(block.id)}><Trash2 className="w-4 h-4 text-destructive" /></Button>
            </div>
          </div>
        </div>
      );
    }
    if (block.type === 'list') {
      return (
        <div key={block.id} draggable onDragStart={handleDragStart} onDragOver={handleDragOver} onDrop={handleDrop} onClick={handleClickSelect} style={{ marginLeft: depth * 24, boxShadow: '0 2px 8px rgba(0,0,0,0.10)', borderRadius: 12, background: '#E6E6FF', border: '2px solid #6D28D9' }} className={`px-4 py-2 mb-2 cursor-move ${highlight}`}>
          <div className="flex items-center justify-between mb-2">
            <div className="flex items-center gap-3">
              <span className="font-bold" style={{ color: '#3b0764' }}>清單：</span>
              {selectedBlockId === block.id ? (
                <Input value={(block as BlockList).listName} onChange={(e) => setParentBlocks(updateBlockById(parentBlocks, block.id, { ...(block as BlockList), listName: e.target.value }))} className="h-7 text-sm bg-[#F4F0FF] rounded-none border-none shadow-none px-2" />
              ) : (
                <span style={{ color: '#3b0764' }}>{(block as BlockList).listName}</span>
              )}
            </div>
            <div className="flex items-center gap-2">
              <Button size="sm" variant="outline" onClick={() => setSelectedBlockId(selectedBlockId === block.id ? null : block.id)}>{selectedBlockId === block.id ? 'Done' : 'Edit'}</Button>
              <Button size="sm" variant="ghost" onClick={() => deleteBlock(block.id)}><Trash2 className="w-4 h-4 text-destructive" /></Button>
            </div>
          </div>
          <div style={{ paddingLeft: 12 }}>
            {(block as BlockList).items.map((it, idx) => (
              <div key={idx} className="flex items-center gap-2 mb-1">
                {selectedBlockId === block.id ? (
                  <Input value={String(it)} onChange={(e) => setParentBlocks(updateBlockById(parentBlocks, block.id, { ...(block as BlockList), items: (block as BlockList).items.map((x, i) => (i === idx ? (isNaN(Number(e.target.value)) ? e.target.value : Number(e.target.value)) : x)) }))} className="h-7 text-sm bg-[#F4F0FF] rounded-none border-none shadow-none px-2 w-48" />
                ) : (
                  <span className="text-sm">{String(it)}</span>
                )}
                {selectedBlockId === block.id && (
                  <Button size="sm" variant="ghost" onClick={() => setParentBlocks(updateBlockById(parentBlocks, block.id, { ...(block as BlockList), items: (block as BlockList).items.filter((_, i) => i !== idx) }))}><Trash2 className="w-4 h-4 text-destructive" /></Button>
                )}
              </div>
            ))}
            {selectedBlockId === block.id && (
              <div className="mt-2">
                <Button size="sm" onClick={() => setParentBlocks(updateBlockById(parentBlocks, block.id, { ...(block as BlockList), items: [...(block as BlockList).items, ''] }))}>新增項目</Button>
              </div>
            )}
          </div>
        </div>
      );
    }
    return null;
  };

  // 序列化積木為純資料結構
  function serializeBlocks(bs: Block[]): unknown[] {
    return bs.map((b) => {
      if (b.type === 'action') return { type: 'action', name: b.name };
      if (b.type === 'if') return { type: 'if', condition: b.condition, then: serializeBlocks(b.then), else: serializeBlocks(b.else) };
      if (b.type === 'repeat') return { type: 'repeat', times: b.times, children: serializeBlocks(b.children) };
      if (b.type === 'forever') return { type: 'forever', children: serializeBlocks(b.children) };
      if (b.type === 'variable') return { type: 'variable', variable: b.variable, value: b.value };
      if (b.type === 'list') return { type: 'list', listName: b.listName, items: b.items };
      return null;
    });
  }

  const saveMission = () => {
    try {
      if (mode === 'advanced') {
        const payload = { blocks: serializeBlocks(blocks), targetType, customTarget, confidence: confidence[0], trigger: { triggerType, triggerValue }, savedAt: Date.now() };
        // Always save locally first
        try { window.localStorage.setItem('missionSetupDraft', JSON.stringify(payload)); } catch (err) { console.warn('localStorage write failed', err); }
        // try POST to backend (best-effort; won't fail UI on network error)
        (async () => {
          try {
            const r = await fetch('/api/missions', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
            if (r.ok) {
              toast.success('進階任務已保存到伺服器');
            } else {
              toast('已儲存於本機 (伺服器回應非 200)');
            }
          } catch (err) {
            console.warn('save to server failed', err);
            toast.success('已儲存於本機');
          }
        })();
      } else {
        const payload = { actions: actions.map((a) => ({ id: a.id, name: a.name })), targetType, customTarget, confidence: confidence[0], trigger: { triggerType, triggerValue }, savedAt: Date.now() };
        try { window.localStorage.setItem('missionSetupDraft', JSON.stringify(payload)); } catch (err) { console.warn('localStorage write failed', err); }
        (async () => {
          try {
            const r = await fetch('/api/missions', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
            if (r.ok) toast.success('簡單任務已保存到伺服器');
            else toast('已儲存於本機 (伺服器回應非 200)');
          } catch (err) {
            console.warn('save to server failed', err);
            toast.success('已儲存於本機');
          }
        })();
      }
    } catch (err) {
      console.error(err);
      toast.error('保存失敗，請查看控制台');
    }
  };

  

  // Move a top-level block to a target index (used by gap drop zones)
  function moveTopLevelBlockToIndex(fromId: string, toIndex: number) {
    setBlocks((prev) => {
      const fromIdx = prev.findIndex((b) => b.id === fromId);
      if (fromIdx === -1) return prev;
      const arr = [...prev];
      const [moved] = arr.splice(fromIdx, 1);
      const insertAt = toIndex > arr.length ? arr.length : toIndex;
      arr.splice(insertAt, 0, moved);
      return arr;
    });
  }

  // find block by id (recursive)
  function findBlockById(bs: Block[], id: string): Block | null {
    for (const b of bs) {
      if (b.id === id) return b;
      if (b.type === 'if') {
        const t = findBlockById(b.then, id);
        if (t) return t;
        const e = findBlockById(b.else, id);
        if (e) return e;
      }
      if (b.type === 'repeat' || b.type === 'forever') {
        const c = findBlockById((b as BlockRepeat | BlockForever).children, id);
        if (c) return c;
      }
    }
    return null;
  }

  return (
    <div className="min-h-screen p-6 space-y-6">
      <div className="flex items-center justify-end mb-4">
        <Button size="sm" className="mr-3" onClick={() => saveMission()}>
          <Save className="mr-2 h-4 w-4" /> 保存
        </Button>
        <Button
          variant={mode === 'simple' ? 'default' : 'outline'}
          className="mr-2"
          onClick={() => setMode('simple')}
        >
          簡單模式
        </Button>
        <Button
          variant={mode === 'advanced' ? 'default' : 'outline'}
          onClick={() => setMode('advanced')}
        >
          進階模式
        </Button>
      </div>
      <div className="max-w-7xl mx-auto">
        {mode === 'simple' ? (
          <div className="flex items-center justify-between mb-8">
            {/* ...原本簡單模式內容... */}
            {/* ...existing code... */}
          </div>
        ) : (
          <div className="flex items-center justify-between mb-8">
            {/* 進階模式內容：積木編輯器、積木類型選單、Undo/Redo/Duplicate/Save 等 */}
            {/* ...existing code... */}
          </div>
        )}

        <div className="grid lg:grid-cols-2 gap-6">
          <Card className="glass p-6 space-y-6">
            <div>
              <h2 className="text-xl font-semibold mb-2">目標物體訓練</h2>
              <p className="text-sm text-muted-foreground">上傳 5 張不同角度的目標物體照片</p>
            </div>

            <div className="grid grid-cols-3 gap-4">
              {images.map((image, index) => (
                <div
                  key={index}
                  className={`relative aspect-square rounded-lg border-2 border-dashed ${
                    image ? "border-accent" : "border-border"
                  } hover:border-accent/50 transition-all overflow-hidden group`}
                >
                  {image ? (
                    <>
                      <img src={image} alt={`Target ${index + 1}`} className="w-full h-full object-cover" />
                      {bboxes[index] && (
                        <div className="absolute left-0 top-0 w-full h-full pointer-events-none">
                          <svg className="w-full h-full" viewBox="0 0 100 100" preserveAspectRatio="none">
                            <rect
                              x={bboxes[index]!.x}
                              y={bboxes[index]!.y}
                              width={bboxes[index]!.width}
                              height={bboxes[index]!.height}
                              fill="rgba(6,182,212,0.12)"
                              stroke="#06b6d4"
                              strokeWidth={0.6}
                            />
                          </svg>
                        </div>
                      )}

                      <button
                        onClick={() => handleRemoveImage(index)}
                        className="absolute top-2 right-2 w-6 h-6 rounded-full bg-destructive/90 flex items-center justify-center opacity-0 group-hover:opacity-100 transition-opacity"
                      >
                        <X className="w-4 h-4" />
                      </button>

                      <button
                        onClick={() => openCanvasEditor(index)}
                        className="absolute left-2 bottom-2 bg-white/80 text-sm px-2 py-1 rounded-md"
                      >
                        編輯標註
                      </button>
                    </>
                  ) : (
                    <label className="w-full h-full flex flex-col items-center justify-center cursor-pointer hover:bg-accent/5">
                      <Upload className="w-8 h-8 text-muted-foreground mb-2" />
                      <span className="text-xs text-muted-foreground">上傳照片</span>
                      <input
                        type="file"
                        accept="image/*"
                        className="hidden"
                        onChange={(e) => e.target.files?.[0] && handleImageUpload(index, e.target.files[0])}
                      />
                    </label>
                  )}
                </div>
              ))}
            </div>

            <div className="space-y-2">
              <Label>目標物體類型</Label>
              <Select value={targetType} onValueChange={setTargetType}>
                <SelectTrigger>
                  <SelectValue placeholder="選擇目標類型" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="fish">魚類</SelectItem>
                  <SelectItem value="coral">珊瑚</SelectItem>
                  <SelectItem value="rock">岩石</SelectItem>
                  <SelectItem value="pipe">管道</SelectItem>
                  <SelectItem value="wreck">殘骸</SelectItem>
                  <SelectItem value="custom">自定義</SelectItem>
                </SelectContent>
              </Select>
              {targetType === "custom" && (
                <div className="pt-2">
                  <Label>自定義名稱</Label>
                  <Input
                    value={customTarget}
                    onChange={(e) => setCustomTarget(e.target.value)}
                    placeholder="請輸入自定義目標名稱"
                  />
                </div>
              )}
            </div>

            <div className="space-y-4">
              <div className="flex items-center justify-between">
                <Label>AI 識別信心度閾值</Label>
                <span className="text-sm font-medium text-accent">{confidence[0]}%</span>
              </div>
              <Slider
                value={confidence}
                onValueChange={setConfidence}
                min={0}
                max={100}
                step={5}
                className="w-full"
              />
              <p className="text-xs text-muted-foreground">當識別信心度高於此值時，系統才會觸發追蹤</p>
            </div>
          </Card>

          <Card className="glass p-6 space-y-6">
            <div className="flex items-center justify-between">
              <div>
                <h2 className="text-xl font-semibold mb-2">動作序列配置</h2>
                <p className="text-sm text-muted-foreground">拖放式動作編輯器</p>
              </div>
              <Button variant="outline" size="sm" className="border-accent/50" onClick={() => setActions([])}>
                <Plus className="mr-2 h-4 w-4" /> 清空序列
              </Button>
            </div>

            {mode === 'advanced' ? (
              <div className="grid grid-cols-3 gap-4">
                <div className="col-span-1">
                  <h4 className="font-medium mb-2">積木庫</h4>
                  <div className="space-y-2">
                    <Button size="sm" className="w-full cursor-grab" draggable onDragStart={(e) => handlePaletteDragStart(e, 'action', { name: '靠近目標' })} onDragEnd={handlePaletteDragEnd} onMouseUp={handlePaletteMouseUp} onClick={paletteClickGuard(() => addBlock(createActionBlock('靠近目標')))}>動作</Button>
                    <Button size="sm" className="w-full cursor-grab" draggable onDragStart={(e) => handlePaletteDragStart(e, 'if', { condition: '目標在視野內' })} onDragEnd={handlePaletteDragEnd} onMouseUp={handlePaletteMouseUp} onClick={paletteClickGuard(() => addBlock(createIfBlock('目標在視野內')))}>If</Button>
                    <Button size="sm" className="w-full cursor-grab" draggable onDragStart={(e) => handlePaletteDragStart(e, 'repeat', { times: 3 })} onDragEnd={handlePaletteDragEnd} onMouseUp={handlePaletteMouseUp} onClick={paletteClickGuard(() => addBlock(createRepeatBlock(3)))}>Repeat</Button>
                    <Button size="sm" className="w-full cursor-grab" draggable onDragStart={(e) => handlePaletteDragStart(e, 'forever')} onDragEnd={handlePaletteDragEnd} onMouseUp={handlePaletteMouseUp} onClick={paletteClickGuard(() => addBlock(createForeverBlock()))}>Forever</Button>
                    <Button size="sm" className="w-full cursor-grab" draggable onDragStart={(e) => handlePaletteDragStart(e, 'variable', { variable: 'x', value: 0 })} onDragEnd={handlePaletteDragEnd} onMouseUp={handlePaletteMouseUp} onClick={paletteClickGuard(() => addBlock(createVariableBlock('x', 0)))}>Variable</Button>
                    <Button size="sm" className="w-full cursor-grab" draggable onDragStart={(e) => handlePaletteDragStart(e, 'list', { listName: 'myList' })} onDragEnd={handlePaletteDragEnd} onMouseUp={handlePaletteMouseUp} onClick={paletteClickGuard(() => addBlock(createListBlock('myList')))}>Make List</Button>
                    <div className="pt-4">
                      <Button size="sm" variant="outline" className="w-full" onClick={() => { setBlocks([]); toast.success('已清空積木'); }}>清空積木</Button>
                    </div>
                  </div>
                </div>
                <div className="col-span-2">
                  <div className="flex items-center justify-between mb-2">
                    <div className="flex items-center gap-2">
                      <Button size="sm" variant="outline" onClick={undo} disabled={undoStack.length === 0}>Undo</Button>
                      <Button size="sm" variant="outline" onClick={redo} disabled={redoStack.length === 0}>Redo</Button>
                      <Button size="sm" variant="outline" onClick={() => selectedBlockId ? duplicateBlockExternal(blocks, setBlocks, selectedBlockId) : toast.error('請先選取一個積木')}>Duplicate</Button>
                    </div>
                    <div className="flex items-center gap-2">
                      <Button size="sm" className="bg-accent" onClick={() => {
                        // serialize blocks
                        function serialize(bs: Block[]): unknown[] {
                          return bs.map((b) => {
                            if (b.type === 'action') return { type: 'action', name: b.name };
                            if (b.type === 'if') return { type: 'if', condition: b.condition, then: serialize(b.then), else: serialize(b.else) };
                            if (b.type === 'repeat') return { type: 'repeat', times: b.times, children: serialize(b.children) };
                            if (b.type === 'forever') return { type: 'forever', children: serialize(b.children) };
                            if (b.type === 'variable') return { type: 'variable', variable: b.variable, value: b.value };
                            if (b.type === 'list') return { type: 'list', listName: b.listName, items: b.items };
                            return null;
                          });
                        }
                        const payload = { blocks: serialize(blocks), targetType, customTarget, confidence: confidence[0], trigger: { triggerType, triggerValue } };
                        console.log('保存進階積木 payload:', payload);
                        toast.success(`進階積木已保存 (${blocks.length})`);
                      }}>
                        <Save className="mr-2 h-4 w-4" /> 儲存
                      </Button>
                    </div>
                  </div>

                  <div className="glass rounded-lg p-4 min-h-[300px] border-2 border-dashed border-border hover:border-accent/50 transition-all relative"
                    onDragOver={(e) => { e.preventDefault(); e.dataTransfer.dropEffect = 'copy'; }}
                    onDrop={(e) => {
                      e.preventDefault();
                      // If canvas empty, accept palette drops and append
                      const paletteData = e.dataTransfer.getData('application/x-block');
                      if (paletteData) {
                        try {
                          const parsed = JSON.parse(paletteData);
                          let newB: Block | null = null;
                          if (parsed.type === 'action') newB = createActionBlock(parsed.payload?.name || '動作');
                          if (parsed.type === 'if') newB = createIfBlock(parsed.payload?.condition || 'condition');
                          if (parsed.type === 'repeat') newB = createRepeatBlock(parsed.payload?.times || 3);
                          if (parsed.type === 'forever') newB = createForeverBlock();
                          if (parsed.type === 'variable') newB = createVariableBlock(parsed.payload?.variable || 'var', parsed.payload?.value ?? 0);
                          if (parsed.type === 'list') newB = createListBlock(parsed.payload?.listName || 'list');
                          if (newB) {
                            setBlocks((prev) => [...prev, newB as Block]);
                            toast.success('已從積木庫新增積木');
                          }
                        } catch (err) {
                          console.warn('Invalid palette drop data', err);
                        }
                        setDragOverGapIndex(null);
                        return;
                      }
                      // otherwise, if dragging existing block, move to end
                      if (dragBlockId) {
                        moveTopLevelBlockToIndex(dragBlockId, blocks.length);
                      }
                      setDragOverGapIndex(null);
                    }}
                  >
                    {blocks.length === 0 ? (
                      <div className="flex items-center justify-center h-40">
                        <p className="text-sm text-muted-foreground">點選左側積木以新增，或拖曳已有積木重新排序</p>
                      </div>
                    ) : (
                      <div className="space-y-2">
                        {blocks.map((b, i) => (
                          <React.Fragment key={b.id}>
                            <div
                              className={`h-3 ${dragOverGapIndex === i ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'} transition-all`}
                              onDragOver={(e) => { e.preventDefault(); setDragOverGapIndex(i); e.dataTransfer.dropEffect = 'move'; }}
                              onDrop={(e) => {
                                e.preventDefault();
                                e.stopPropagation();
                                const paletteData = e.dataTransfer.getData('application/x-block');
                                if (paletteData) {
                                  try {
                                    const parsed = JSON.parse(paletteData);
                                    let newB: Block | null = null;
                                    if (parsed.type === 'action') newB = createActionBlock(parsed.payload?.name || '動作');
                                    if (parsed.type === 'if') newB = createIfBlock(parsed.payload?.condition || 'condition');
                                    if (parsed.type === 'repeat') newB = createRepeatBlock(parsed.payload?.times || 3);
                                    if (parsed.type === 'forever') newB = createForeverBlock();
                                    if (parsed.type === 'variable') newB = createVariableBlock(parsed.payload?.variable || 'var', parsed.payload?.value ?? 0);
                                    if (parsed.type === 'list') newB = createListBlock(parsed.payload?.listName || 'list');
                                    if (newB) {
                                      setBlocks((prev) => {
                                        const arr = [...prev];
                                        arr.splice(i, 0, newB as Block);
                                        return arr;
                                      });
                                      toast.success('已從積木庫新增積木');
                                    }
                                  } catch (err) {
                                    console.warn('Invalid palette drop data', err);
                                  }
                                  setDragOverGapIndex(null);
                                  return;
                                }
                                // otherwise move existing top-level block
                                if (dragBlockId) {
                                  moveTopLevelBlockToIndex(dragBlockId, i);
                                }
                                setDragOverGapIndex(null);
                              }}
                            />
                            {renderBlock(b)}
                          </React.Fragment>
                        ))}
                        <div
                          className={`h-6 ${dragOverGapIndex === blocks.length ? 'bg-cyan-200 rounded-sm' : 'bg-transparent'} transition-all`}
                          onDragOver={(e) => { e.preventDefault(); setDragOverGapIndex(blocks.length); e.dataTransfer.dropEffect = 'move'; }}
                          onDrop={(e) => {
                            e.preventDefault();
                            e.stopPropagation();
                            const paletteData = e.dataTransfer.getData('application/x-block');
                            if (paletteData) {
                              try {
                                const parsed = JSON.parse(paletteData);
                                let newB: Block | null = null;
                                if (parsed.type === 'action') newB = createActionBlock(parsed.payload?.name || '動作');
                                if (parsed.type === 'if') newB = createIfBlock(parsed.payload?.condition || 'condition');
                                if (parsed.type === 'repeat') newB = createRepeatBlock(parsed.payload?.times || 3);
                                if (parsed.type === 'forever') newB = createForeverBlock();
                                if (parsed.type === 'variable') newB = createVariableBlock(parsed.payload?.variable || 'var', parsed.payload?.value ?? 0);
                                if (parsed.type === 'list') newB = createListBlock(parsed.payload?.listName || 'list');
                                if (newB) {
                                  setBlocks((prev) => [...prev, newB as Block]);
                                  toast.success('已從積木庫新增積木 (末端)');
                                }
                              } catch (err) {
                                console.warn('Invalid palette drop data', err);
                              }
                              setDragOverGapIndex(null);
                              return;
                            }
                            if (dragBlockId) {
                              moveTopLevelBlockToIndex(dragBlockId, blocks.length);
                            }
                            setDragOverGapIndex(null);
                          }}
                        />
                      </div>
                    )}
                    {/* Centered inline inspector overlay inside canvas */}
                    {selectedBlockId && (() => {
                      const selectedBlock = findBlockById(blocks, selectedBlockId);
                      if (!selectedBlock) return null;
                      return (
                        <div className="absolute inset-0 flex items-center justify-center pointer-events-none">
                          <div className="w-96 bg-white/95 border p-4 rounded-lg shadow-lg z-40 pointer-events-auto">
                            <div className="flex items-center justify-between mb-2">
                              <div className="font-semibold">積木編輯</div>
                              <Button size="sm" variant="ghost" onClick={() => setSelectedBlockId(null)}>關閉</Button>
                            </div>
                            <div className="space-y-3 text-sm">
                              <div className="text-xs text-muted-foreground">ID: {selectedBlock.id}</div>
                              <div className="font-medium">類型：{selectedBlock.type}</div>
                              {selectedBlock.type === 'action' && (
                                <div>
                                  <Label>動作名稱</Label>
                                  <Input value={(selectedBlock as BlockAction).name} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { name: e.target.value }))} />
                                </div>
                              )}
                              {selectedBlock.type === 'if' && (
                                <div>
                                  <Label>條件</Label>
                                  <Input value={(selectedBlock as BlockIf).condition} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { condition: e.target.value }))} />
                                </div>
                              )}
                              {selectedBlock.type === 'repeat' && (
                                <div>
                                  <Label>次數</Label>
                                  <Input type="number" value={String((selectedBlock as BlockRepeat).times)} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { times: Number(e.target.value) }))} />
                                </div>
                              )}
                              {selectedBlock.type === 'variable' && (
                                <div>
                                  <Label>變數</Label>
                                  <Input value={(selectedBlock as BlockVariable).variable} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { variable: e.target.value }))} />
                                  <Label className="pt-2">數值</Label>
                                  <Input value={String((selectedBlock as BlockVariable).value)} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { value: isNaN(Number(e.target.value)) ? e.target.value : Number(e.target.value) }))} />
                                </div>
                              )}
                              {selectedBlock.type === 'list' && (
                                <div>
                                  <Label>清單名稱</Label>
                                  <Input value={(selectedBlock as BlockList).listName} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { listName: e.target.value }))} />
                                </div>
                              )}
                            </div>
                          </div>
                        </div>
                      );
                    })()}
                  </div>
                </div>
              </div>
            ) : (
              <>
                <div className="space-y-3">
                  <Label>預設動作庫</Label>
                  <div className="grid grid-cols-2 gap-2">
                    {actionLibrary.map((action) => (
                      <div key={action} className="flex gap-2">
                        <Button
                          variant="outline"
                          size="sm"
                          className="flex-1 justify-start hover:bg-accent/10 hover:border-accent/50 cursor-move"
                          draggable
                          onDragStart={(e) => handleDragStart(e, action)}
                          onDragEnd={() => setDraggedAction(null)}
                        >
                          {action}
                        </Button>
                        <Button
                          variant="outline"
                          size="sm"
                          className="hover:bg-accent/10 hover:border-accent/50"
                          onClick={() => handleAddAction(action)}
                        >
                          +
                        </Button>
                      </div>
                    ))}
                  </div>
                  <p className="text-xs text-muted-foreground">💡 提示: 可以直接點擊 + 按鈕或拖曳動作到時間軸</p>
                </div>

                <div className="space-y-3">
                  <Label>動作時間軸 ({actions.length} 個動作)</Label>
                  <div
                    className="glass rounded-lg p-4 min-h-[250px] border-2 border-dashed border-border hover:border-accent/50 transition-all"
                    onDragOver={handleDragOver}
                    onDrop={handleDrop}
                  >
                    {actions.length === 0 ? (
                      <div className="flex items-center justify-center h-full">
                        <p className="text-sm text-muted-foreground">將動作拖放至此處或點擊 + 按鈕建立序列</p>
                      </div>
                    ) : (
                      <div className="space-y-2">
                        {actions.map((action, index) => (
                          <div
                            key={action.id}
                            className="flex items-center gap-2 p-3 rounded-lg bg-accent/10 border border-accent/50 hover:bg-accent/20 transition-all"
                          >
                            <GripVertical className="w-4 h-4 text-muted-foreground flex-shrink-0" />
                            <span className="text-sm font-medium flex-1">
                              {index + 1}. {action.name}
                            </span>
                            <Button
                              variant="ghost"
                              size="sm"
                              onClick={() => handleRemoveAction(action.id)}
                              className="h-6 w-6 p-0 hover:bg-destructive/10"
                            >
                              <Trash2 className="w-3 h-3 text-destructive" />
                            </Button>
                          </div>
                        ))}
                      </div>
                    )}
                  </div>
                </div>
              </>
            )}

            <div className="space-y-2">
              <Label>觸發條件</Label>
              <Select value={triggerType} onValueChange={setTriggerType}>
                <SelectTrigger>
                  <SelectValue placeholder="選擇觸發條件" />
                </SelectTrigger>
                <SelectContent>
                  <SelectItem value="target-found">發現目標時</SelectItem>
                  <SelectItem value="distance">距離目標 X 米時</SelectItem>
                  <SelectItem value="depth">深度達到 X 米時</SelectItem>
                  <SelectItem value="battery">電量低於 X% 時</SelectItem>
                </SelectContent>
              </Select>

              {triggerType === "distance" && (
                <div className="pt-2">
                  <Label>距離 (米)</Label>
                  <Input
                    type="number"
                    value={triggerValue}
                    onChange={(e) => setTriggerValue(e.target.value)}
                    placeholder="請輸入距離"
                  />
                </div>
              )}

              {triggerType === "depth" && (
                <div className="pt-2">
                  <Label>深度 (米)</Label>
                  <Input
                    type="number"
                    value={triggerValue}
                    onChange={(e) => setTriggerValue(e.target.value)}
                    placeholder="請輸入深度"
                  />
                </div>
              )}

              {triggerType === "battery" && (
                <div className="pt-2">
                  <Label>電量 (%)</Label>
                  <Input
                    type="number"
                    value={triggerValue}
                    onChange={(e) => setTriggerValue(e.target.value)}
                    placeholder="請輸入電量百分比"
                  />
                </div>
              )}
            </div>
          </Card>
        </div>
      </div>

      {/* {selectedBlockId && (
        (() => {
          const selectedBlock = findBlockById(blocks, selectedBlockId);
          if (!selectedBlock) return null;
          return (
            <div className="fixed right-6 top-24 w-80 bg-white/95 border p-4 rounded-lg shadow-lg z-50">
              <div className="flex items-center justify-between mb-2">
                <div className="font-semibold">積木編輯</div>
                <Button size="sm" variant="ghost" onClick={() => setSelectedBlockId(null)}>關閉</Button>
              </div>
              <div className="space-y-3 text-sm">
                <div className="text-xs text-muted-foreground">ID: {selectedBlock.id}</div>
                <div className="font-medium">類型：{selectedBlock.type}</div>
                {selectedBlock.type === 'action' && (
                  <div>
                    <Label>動作名稱</Label>
                    <Input value={(selectedBlock as BlockAction).name} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { name: e.target.value }))} />
                  </div>
                )}
                {selectedBlock.type === 'if' && (
                  <div>
                    <Label>條件</Label>
                    <Input value={(selectedBlock as BlockIf).condition} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { condition: e.target.value }))} />
                  </div>
                )}
                {selectedBlock.type === 'repeat' && (
                  <div>
                    <Label>次數</Label>
                    <Input type="number" value={String((selectedBlock as BlockRepeat).times)} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { times: Number(e.target.value) }))} />
                  </div>
                )}
                {selectedBlock.type === 'variable' && (
                  <div>
                    <Label>變數名稱</Label>
                    <Input value={(selectedBlock as BlockVariable).variable} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { variable: e.target.value }))} />
                    <Label className="pt-2">值</Label>
                    <Input value={String((selectedBlock as BlockVariable).value)} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { value: isNaN(Number(e.target.value)) ? e.target.value : Number(e.target.value) }))} />
                  </div>
                )}
                {selectedBlock.type === 'list' && (
                  <div>
                    <Label>列表名稱</Label>
                    <Input value={(selectedBlock as BlockList).listName} onChange={(e) => setBlocks(updateBlockById(blocks, selectedBlock.id, { listName: e.target.value }))} />
                    <div className="pt-2">
                      <Label>項目</Label>
                      {(selectedBlock as BlockList).items.map((it, idx) => (
                        <div key={idx} className="flex gap-2 mt-2">
                          <Input value={String(it)} onChange={(e) => {
                            const copy = [...(selectedBlock as BlockList).items];
                            copy[idx] = isNaN(Number(e.target.value)) ? e.target.value : Number(e.target.value);
                            setBlocks(updateBlockById(blocks, selectedBlock.id, { items: copy }));
                          }} />
                          <Button size="sm" variant="ghost" onClick={() => {
                            const copy = [...(selectedBlock as BlockList).items];
                            copy.splice(idx, 1);
                            setBlocks(updateBlockById(blocks, selectedBlock.id, { items: copy }));
                          }}>刪除</Button>
                        </div>
                      ))}
                      <Button size="sm" className="mt-2" onClick={() => {
                        const copy = [...(selectedBlock as BlockList).items, ''];
                        setBlocks(updateBlockById(blocks, selectedBlock.id, { items: copy }));
                      }}>新增項目</Button>
                    </div>
                  </div>
                )}
                <div className="pt-3 flex justify-end">
                  <Button size="sm" variant="destructive" onClick={() => { deleteBlock(selectedBlock.id); setSelectedBlockId(null); }}>刪除積木</Button>
                </div>
              </div>
            </div>
          );
        })()
      )} */}

      {canvasOpen && canvasImageIndex !== null && (
        <div className="fixed inset-0 bg-black/40 z-50 flex items-center justify-center">
          <div className="bg-white rounded-lg shadow-lg p-6 relative w-[800px] h-[600px] flex flex-col">
            <button className="absolute top-2 right-2 bg-gray-200 rounded-full w-8 h-8 flex items-center justify-center" onClick={closeCanvasEditor}>
              <X className="w-5 h-5" />
            </button>
            <div className="flex-1 flex items-stretch gap-4">
              <div className="flex-1 flex items-center justify-center bg-gray-50 p-4">
                <img src={images[canvasImageIndex] || undefined} alt="編輯" className="max-w-full max-h-full object-contain" />
              </div>
              <div className="w-72 p-4 border-l flex flex-col gap-4">
                <h3 className="font-semibold mb-2">標註編輯器</h3>
                <p className="text-sm text-muted-foreground mb-2">可新增/刪除/拖曳/縮放標註框</p>
                <Button variant="default" className="w-full">+ 新增框</Button>
                <Button variant="destructive" className="w-full">刪除選取</Button>
                <div className="flex gap-2 mt-4">
                  <Button variant="default" onClick={() => { closeCanvasEditor(); toast.success('標註已保存'); }}>保存</Button>
                  <Button variant="outline" onClick={closeCanvasEditor}>取消</Button>
                </div>
                <div className="mt-2 text-xs text-muted-foreground">尚未選取任何框，點擊或在列表中選取一個。</div>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default MissionSetup;

// 移除積木（遞迴）
function removeBlockById(blocks: Block[], id: string): Block[] {
  return blocks
    .filter((b) => b.id !== id)
    .map((b) => {
      if (b.type === "if") {
        return { ...b, then: removeBlockById(b.then, id), else: removeBlockById(b.else, id) };
      }
      if (b.type === "repeat") {
        return { ...b, children: removeBlockById(b.children, id) };
      }
      return b;
    });
}

// 插入積木到嵌套區
function insertBlock(blocks: Block[], targetId: string, block: Block, branch: "then" | "else" | "children"): Block[] {
  return blocks.map((b) => {
    if (b.id === targetId) {
      if (b.type === "if" && (branch === "then" || branch === "else")) {
        return {
          ...b,
          [branch]: [...b[branch], block],
        };
      }
      if (b.type === "repeat" && branch === "children") {
        return {
          ...b,
          children: [...b.children, block],
        };
      }
    }
    if (b.type === "if") {
      return { ...b, then: insertBlock(b.then, targetId, block, branch), else: insertBlock(b.else, targetId, block, branch) };
    }
    if (b.type === "repeat") {
      return { ...b, children: insertBlock(b.children, targetId, block, branch) };
    }
    return b;
  });
}

// 插入積木到分支的指定位置
function insertBlockAt(blocks: Block[], targetId: string, block: Block, branch: "then" | "else" | "children", index: number): Block[] {
  return blocks.map((b) => {
    if (b.id === targetId) {
      if (b.type === 'if' && (branch === 'then' || branch === 'else')) {
        const arr = [...b[branch]];
        const insertAt = Math.max(0, Math.min(index, arr.length));
        arr.splice(insertAt, 0, block);
        return { ...b, [branch]: arr } as BlockIf;
      }
      if ((b.type === 'repeat' || b.type === 'forever') && branch === 'children') {
        const arr = [...(b as BlockRepeat | BlockForever).children];
        const insertAt = Math.max(0, Math.min(index, arr.length));
        arr.splice(insertAt, 0, block);
        if (b.type === 'repeat') return { ...(b as BlockRepeat), children: arr } as BlockRepeat;
        return { ...(b as BlockForever), children: arr } as BlockForever;
      }
    }
    if (b.type === 'if') {
      return { ...b, then: insertBlockAt(b.then, targetId, block, branch, index), else: insertBlockAt(b.else, targetId, block, branch, index) };
    }
    if (b.type === 'repeat') {
      return { ...b, children: insertBlockAt(b.children, targetId, block, branch, index) };
    }
    return b;
  });
}

// 判斷 candidateId 是否在 ancestorId 的子孫中
function isDescendant(bs: Block[], ancestorId: string, candidateId: string): boolean {
  let ancestor: Block | null = null;
  function find(bArr: Block[]) {
    for (const b of bArr) {
      if (b.id === ancestorId) {
        ancestor = b;
        return true;
      }
      if (b.type === 'if') {
        if (find(b.then)) return true;
        if (find(b.else)) return true;
      }
      if (b.type === 'repeat' || b.type === 'forever') {
        if (find((b as BlockRepeat | BlockForever).children)) return true;
      }
    }
    return false;
  }
  find(bs);
  if (!ancestor) return false;
  let found = false;
  function walk(b: Block) {
    if (b.id === candidateId) {
      found = true;
      return;
    }
    if (b.type === 'if') {
      b.then.forEach(walk);
      b.else.forEach(walk);
    }
    if (b.type === 'repeat' || b.type === 'forever') {
      (b as BlockRepeat | BlockForever).children.forEach(walk);
    }
  }
  walk(ancestor);
  return found;
}

// 找到父陣列與索引位置，用於複製/插入
function findParentAndIndex(bs: Block[], id: string, parent: Block[] | null = null): { parentArr: Block[] | null; index: number } | null {
  for (let i = 0; i < bs.length; i++) {
    const b = bs[i];
    if (b.id === id) return { parentArr: parent, index: i };
    if (b.type === 'if') {
      const r = findParentAndIndex(b.then, id, b.then);
      if (r) return r;
      const r2 = findParentAndIndex(b.else, id, b.else);
      if (r2) return r2;
    }
    if (b.type === 'repeat') {
      const r = findParentAndIndex(b.children, id, b.children);
      if (r) return r;
    }
  }
  return null;
}

function duplicateBlockExternal(blocksParam: Block[], setBlocksParam: React.Dispatch<React.SetStateAction<Block[]>>, id: string) {
  const info = findParentAndIndex(blocksParam, id);
  if (!info) return;
  const parentArr = info.parentArr || blocksParam;
  const original = parentArr[info.index];
  const copy = typeof structuredClone !== 'undefined' ? (structuredClone(original) as Block) : JSON.parse(JSON.stringify(original));
  // assign new ids recursively
  function reid(b: Block): Block {
    const newId = `${b.type}-${Date.now()}-${Math.random().toString(16).slice(2,6)}`;
    if (b.type === 'action') return { ...(b as BlockAction), id: newId } as BlockAction;
    if (b.type === 'if') return { ...(b as BlockIf), id: newId, then: b.then.map(reid), else: b.else.map(reid) } as BlockIf;
    if (b.type === 'repeat') return { ...(b as BlockRepeat), id: newId, children: b.children.map(reid) } as BlockRepeat;
    return b;
  }
  const newBlock = reid(copy as Block);
  // insert after
  const newBlocks = (function insertAt(bs: Block[]): Block[] {
    if (info.parentArr === null) {
      const arr = [...bs];
      arr.splice(info.index + 1, 0, newBlock);
      return arr;
    }
    // recursive insertion into nested structure
    return bs.map((b) => {
      if (b.type === 'if') {
        return { ...b, then: insertAt(b.then), else: insertAt(b.else) };
      }
      if (b.type === 'repeat') {
        return { ...b, children: insertAt(b.children) };
      }
      return b;
    });
  })(blocksParam);
  setBlocksParam(newBlocks);
  toast.success('已複製積木');
}

// 找到父積木的 id（若為頂層則返回 null）
function findParentBlockId(bs: Block[], id: string, parentId: string | null = null): string | null {
  for (const b of bs) {
    if (b.id === id) return parentId;
    if (b.type === 'if') {
      const r = findParentBlockId(b.then, id, b.id);
      if (r !== null) return r;
      const r2 = findParentBlockId(b.else, id, b.id);
      if (r2 !== null) return r2;
    }
    if (b.type === 'repeat' || b.type === 'forever') {
      const r = findParentBlockId((b as BlockRepeat | BlockForever).children, id, b.id);
      if (r !== null) return r;
    }
  }
  return null;
}

