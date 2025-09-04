import math
import random
import time
import threading
import heapq
from concurrent.futures import ThreadPoolExecutor
from collections import defaultdict
import json
from tkinter import filedialog

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog


# Graph Model

class Graph:
    def __init__(self):
        self.vertices = {}
        self.adj = defaultdict(dict)

    def add_vertex(self, name, x, y):
        if name in self.vertices:
            raise ValueError(f"Vertex '{name}' already exists.")
        self.vertices[name] = {'x': float(x), 'y': float(y)}

    def delete_vertex(self, name):
        if name not in self.vertices:
            raise ValueError(f"Vertex '{name}' does not exist.")
        for neigh in list(self.adj[name].keys()):
            self.delete_edge(name, neigh)
        for u in list(self.adj.keys()):
            if name in self.adj[u]:
                del self.adj[u][name]
        del self.vertices[name]
        if name in self.adj:
            del self.adj[name]






# adding edge

    def add_edge(self, u, v, fixed=None):
        if u == v:
            raise ValueError("Self-loops are not allowed.")
        if u not in self.vertices or v not in self.vertices:
            raise ValueError("Both vertices must exist.")
        if fixed is None:
            fixed = self.euclidean(u, v)
        fixed = float(fixed)
        self.adj[u][v] = {'fixed': fixed, 'dynamic': 0.0}
        self.adj[v][u] = {'fixed': fixed, 'dynamic': 0.0}







    def delete_edge(self, u, v):
        if u in self.adj and v in self.adj[u]:
            del self.adj[u][v]
        if v in self.adj and u in self.adj[v]:
            del self.adj[v][u]

    def set_fixed(self, u, v, fixed):
        if v not in self.adj.get(u, {}):
            raise ValueError("Edge does not exist.")
        self.adj[u][v]['fixed'] = float(fixed)
        self.adj[v][u]['fixed'] = float(fixed)

    def set_dynamic(self, u, v, dynamic):
        if v not in self.adj.get(u, {}):
            raise ValueError("Edge does not exist.")
        self.adj[u][v]['dynamic'] = float(dynamic)
        self.adj[v][u]['dynamic'] = float(dynamic)

    def get_neighbors(self, u):
        return list(self.adj.get(u, {}).keys())

    def effective_weight(self, u, v):
        e = self.adj[u][v]
        return e['fixed'] + (e.get('dynamic') or 0.0)





# Distane calculated between each nodes

    def euclidean(self, u, v):
        ax, ay = self.vertices[u]['x'], self.vertices[u]['y']
        bx, by = self.vertices[v]['x'], self.vertices[v]['y']
        return math.hypot(ax - bx, ay - by)





    def heuristic(self, u, goal):
        return self.euclidean(u, goal)

    def randomize_dynamic(self, subset_pct=20, seed=None, dist_params=None):
        rng = random.Random(seed)
        edges = []
        seen = set()
        for u in self.adj:
            for v in self.adj[u]:
                key = tuple(sorted((u, v)))
                if key in seen: continue
                seen.add(key)
                edges.append(key)
        rng.shuffle(edges)
        target = max(0, min(len(edges), int(len(edges) * (subset_pct / 100.0))))

        dist_params = dist_params or {
            'probs': (0.6, 0.3, 0.1),
            'small': (2, 5),
            'medium': (6, 12),
            'large': (13, 25),
        }
        p_small, p_med, p_large = dist_params['probs']
        choices = []
        for _ in range(target):
            r = rng.random()
            if r < p_small:
                a, b = dist_params['small']
            elif r < p_small + p_med:
                a, b = dist_params['medium']
            else:
                a, b = dist_params['large']
            choices.append(rng.uniform(a, b))

        for u, vdict in self.adj.items():
            for v in vdict:
                self.adj[u][v]['dynamic'] = 0.0
        for i in range(target):
            u, v = edges[i]
            self.set_dynamic(u, v, float(choices[i]))

    def connect_k_nearest(self, k=3):
        names = list(self.vertices.keys())
       
        parent = {n: n for n in names}
        def find(u):
            if parent[u] != u:
                parent[u] = find(parent[u])
            return parent[u]
        def union(u, v):
            parent[find(u)] = find(v)
        
        edges = []
        for i, u in enumerate(names):
            for v in names[i+1:]:
                dist = self.euclidean(u, v)
                edges.append((dist, u, v))
        edges.sort()
        for dist, u, v in edges:
            if find(u) != find(v):
                self.add_edge(u, v, dist)
                union(u, v)
        
        # Add k-nearest for additional connectivity
        for u in names:
            dists = [(self.euclidean(u, v), v) for v in names if u != v]
            dists.sort()
            for _, v in dists[:k]:
                if v not in self.adj[u]:
                    self.add_edge(u, v)

    def get_connected_components(self):
        parent = {n: n for n in self.vertices}
        rank = {n: 0 for n in self.vertices}
        def find(u):
            if parent[u] != u:
                parent[u] = find(parent[u])
            return parent[u]
        def union(u, v):
            pu, pv = find(u), find(v)
            if pu == pv:
                return
            if rank[pu] < rank[pv]:
                pu, pv = pv, pu
            parent[pv] = pu
            if rank[pu] == rank[pv]:
                rank[pu] += 1
        for u in self.adj:
            for v in self.adj[u]:
                union(u, v)
        components = defaultdict(list)
        for n in self.vertices:
            components[find(n)].append(n)
        return list(components.values())


# Pathfinding Algorithms

    # A*
def astar(graph: Graph, start: str, goal: str, stop_event: threading.Event = None, log_fn=None):
    start_t = time.perf_counter()
    open_heap = [(0.0, 0.0, start, [start])]  # (f, g, node, path)
    heapq.heapify(open_heap)
    visited = set()
    sample_paths = []

    while open_heap:
        if stop_event and stop_event.is_set():
            if log_fn:
                log_fn("A* stopped early due to cancellation.")
            break
        f, g, node, path = heapq.heappop(open_heap)
        visited.add(node)
        if len(sample_paths) < 20:
            sample_paths.append(list(path))

        if node == goal:
            runtime = (time.perf_counter() - start_t) * 1000.0
            return path, g, runtime, visited, sample_paths[:10]

        for neigh in graph.get_neighbors(node):
            if neigh in path:
                continue
            w = graph.effective_weight(node, neigh)
            g2 = g + w
            h = graph.heuristic(neigh, goal)
            f2 = g2 + h
            heapq.heappush(open_heap, (f2, g2, neigh, path + [neigh]))

    runtime = (time.perf_counter() - start_t) * 1000.0
    if log_fn:
        log_fn(f"A* failed: no path found. Visited {len(visited)} nodes, heap size {len(open_heap)}.")
    return None, float('inf'), runtime, visited, sample_paths[:10]



    # dijkstra

def dijkstra(graph: Graph, start: str, goal: str, stop_event: threading.Event = None, log_fn=None):
    start_t = time.perf_counter()
    open_heap = [(0.0, start, [start])]  # (g, node, path)
    heapq.heapify(open_heap)
    visited = set()
    sample_paths = []

    while open_heap:
        if stop_event and stop_event.is_set():
            if log_fn:
                log_fn("Dijkstra stopped early due to cancellation.")
            break
        g, node, path = heapq.heappop(open_heap)
        if node in visited:
            continue
        visited.add(node)
        if len(sample_paths) < 20:
            sample_paths.append(list(path))

        if node == goal:
            runtime = (time.perf_counter() - start_t) * 1000.0
            return path, g, runtime, visited, sample_paths[:10]

        for neigh in graph.get_neighbors(node):
            if neigh in path:
                continue
            w = graph.effective_weight(node, neigh)
            g2 = g + w
            heapq.heappush(open_heap, (g2, neigh, path + [neigh]))

    runtime = (time.perf_counter() - start_t) * 1000.0
    if log_fn:
        log_fn(f"Dijkstra failed: no path found. Visited {len(visited)} nodes, heap size {len(open_heap)}.")
    return None, float('inf'), runtime, visited, sample_paths[:10]



    # Brute-force

def brute_force_all_paths(graph: Graph, start: str, goal: str, cap: int = 50000, demo_nodes=None, stop_event: threading.Event = None, log_fn=None):
    start_t = time.perf_counter()
    allowed = set(graph.vertices.keys()) if demo_nodes is None else set(demo_nodes)
    paths_explored = 0
    sample_paths = []
    best_path = None
    best_cost = float('inf')

    stack = [(start, iter([n for n in graph.get_neighbors(start) if n in allowed]), [start], 0.0)]
    while stack:
        if stop_event and stop_event.is_set():
            if log_fn:
                log_fn("Brute Force stopped early due to cancellation.")
            break
        node, it, path, cost = stack[-1]
        try:
            neigh = next(it)
            if neigh in path or neigh not in allowed:
                continue
            w = graph.effective_weight(node, neigh)
            new_cost = cost + w
            if neigh == goal:
                paths_explored += 1
                cand = path + [neigh]
                if len(sample_paths) < 200:
                    sample_paths.append(cand)
                if new_cost < best_cost:
                    best_cost = new_cost
                    best_path = cand
                if paths_explored >= cap:
                    if log_fn:
                        log_fn(f"Brute Force stopped: reached cap of {cap} paths.")
                    break
            else:
                stack.append((neigh, iter([n for n in graph.get_neighbors(neigh) if n in allowed]), path + [neigh], new_cost))
        except StopIteration:
            stack.pop()

    runtime = (time.perf_counter() - start_t) * 1000.0
    if log_fn and not best_path:
        log_fn(f"Brute Force failed: no path found. Explored {paths_explored} paths.")
    return best_path, best_cost, runtime, paths_explored, sample_paths[:10]




# Visualization

class Visualizer:
    def __init__(self, canvas: tk.Canvas, graph: Graph):
        self.canvas = canvas
        self.graph = graph
        self.node_radius = 14
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.items_nodes = {}
        self.items_edges = {}
        self.items_edge_labels = {}
        self.path_items = []
        self.legend_ids = []

    def set_zoom(self, zoom):
        self.zoom = max(0.5, min(3.0, zoom))
        self.redraw_all()

    def clear(self):
        for it in self.path_items:
            self.canvas.delete(it)
        self.path_items.clear()

    def redraw_all(self, highlight=None, start=None, goal=None):
        self.canvas.delete("all")
        self.items_nodes.clear()
        self.items_edges.clear()
        self.items_edge_labels.clear()
        self.legend_ids.clear()
        self.clear()

        self._force_nudge(iterations=200)

        w = int(self.canvas.cget("width"))
        h = int(self.canvas.cget("height"))
        self.center_x = w / 2
        self.center_y = h / 2

        drawn = set()
        for u in self.graph.adj:
            for v in self.graph.adj[u]:
                key = tuple(sorted((u, v)))
                if key in drawn:
                    continue
                drawn.add(key)
                self._draw_edge(u, v)

        for name in self.graph.vertices:
            self._draw_node(name, fill="#ffffff", outline="#333333")

        if start and start in self.items_nodes:
            self._color_node(start, fill="#c8e6c9", outline="#388e3c")
        if goal and goal in self.items_nodes:
            self._color_node(goal, fill="#ffcdd2", outline="#d32f2f")

        self._draw_legend()

    def _force_nudge(self, iterations=200):
        names = list(self.graph.vertices.keys())
        if not names:
            return
        try:
            w = int(self.canvas.cget("width"))
            h = int(self.canvas.cget("height"))
        except Exception:
            w, h = 1000, 700

        for _ in range(iterations):
            disp = {n: [0.0, 0.0] for n in names}
            for i, a in enumerate(names):
                ax, ay = self.graph.vertices[a]['x'], self.graph.vertices[a]['y']
                for b in names[i+1:]:
                    bx, by = self.graph.vertices[b]['x'], self.graph.vertices[b]['y']
                    dx, dy = ax - bx, ay - by
                    dist = math.hypot(dx, dy) + 1e-6
                    if dist < 100:
                        force = 100.0 / dist
                        fx, fy = force * (dx / dist), force * (dy / dist)
                        disp[a][0] += fx; disp[a][1] += fy
                        disp[b][0] -= fx; disp[b][1] -= fy
            for u in self.graph.adj:
                for v in self.graph.adj[u]:
                    if u < v:
                        ux, uy = self.graph.vertices[u]['x'], self.graph.vertices[u]['y']
                        vx, vy = self.graph.vertices[v]['x'], self.graph.vertices[v]['y']
                        dx, dy = vx - ux, vy - uy
                        dist = math.hypot(dx, dy) + 1e-6
                        if dist > 150:
                            force = (dist - 150) * 0.05
                            fx, fy = force * (dx / dist), force * (dy / dist)
                            disp[u][0] += fx; disp[u][1] += fy
                            disp[v][0] -= fx; disp[v][1] -= fy
            for n in names:
                self.graph.vertices[n]['x'] = max(30, min(w-30, self.graph.vertices[n]['x'] + disp[n][0]))
                self.graph.vertices[n]['y'] = max(30, min(h-30, self.graph.vertices[n]['y'] + disp[n][1]))

    def _scaled_coords(self, x, y):
        sx = self.center_x + (x - self.pan_x) * self.zoom
        sy = self.center_y + (y - self.pan_y) * self.zoom
        return sx, sy

    def _curve_offset_for(self, u, v):
        ax, ay = self.graph.vertices[u]['x'], self.graph.vertices[u]['y']
        bx, by = self.graph.vertices[v]['x'], self.graph.vertices[v]['y']
        mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
        dx, dy = bx - ax, by - ay
        dist = math.hypot(dx, dy) + 1e-6
        px, py = -dy / dist, dx / dist
        base = 22.0 + (hash((u, v)) % 10)
        return mx + px * base, my + py * base

    def _draw_edge(self, u, v):
        key = tuple(sorted((u, v)))
        if key in self.items_edges:
            return
        ax, ay = self._scaled_coords(self.graph.vertices[u]['x'], self.graph.vertices[u]['y'])
        bx, by = self._scaled_coords(self.graph.vertices[v]['x'], self.graph.vertices[v]['y'])
        mx, my = self._curve_offset_for(u, v)
        mx, my = self._scaled_coords(mx, my)

        e = self.graph.adj[u][v]
        fixed = e['fixed']
        dynamic = e.get('dynamic', 0.0)
        is_dynamic = dynamic and dynamic > 1e-9

        width = 2 * self.zoom
        font_size = int(9 * self.zoom)
        line = self.canvas.create_line(ax, ay, mx, my, bx, by,
                                       smooth=True, width=width,
                                       fill="#bdbdbd" if not is_dynamic else "#ffa726",
                                       dash=() if not is_dynamic else (6, 4),
                                       tags=("movable",))
        self.items_edges[key] = line





# weight displayed


        label = f"{fixed:.1f}" if not is_dynamic else f"{fixed:.1f} | +{dynamic:.1f}"
        tx, ty = mx, my
        tid = self.canvas.create_text(tx, ty, text=label, fill="#616161",
                                     font=("Helvetica", font_size, "bold"), tags=("movable",))
        self.items_edge_labels[key] = tid




    def _draw_node(self, name, fill="white", outline="#444"):
        x, y = self.graph.vertices[name]['x'], self.graph.vertices[name]['y']
        sx, sy = self._scaled_coords(x, y)
        r = self.node_radius * self.zoom
        font_size = int(9 * self.zoom)
        oval = self.canvas.create_oval(sx-r, sy-r, sx+r, sy+r, fill=fill, outline=outline,
                                      width=2 * self.zoom, tags=("movable",))
        text = self.canvas.create_text(sx, sy - (r + 10 * self.zoom), text=name, fill="#212121",
                                      font=("Helvetica", font_size), tags=("movable",))
        self.items_nodes[name] = (oval, text)

    def _color_node(self, name, fill, outline):
        if name in self.items_nodes:
            oval, _ = self.items_nodes[name]
            self.canvas.itemconfigure(oval, fill=fill, outline=outline)

    def _draw_legend(self):
        x0, y0 = 10, 10
        pad = 4 * self.zoom
        font_size = int(9 * self.zoom)
        bold_font_size = int(10 * self.zoom)
        lines = [
            ("Legend", None),
            ("Start", "#c8e6c9"),
            ("Goal", "#ffcdd2"),
            ("Dynamic edge", "#ffa726"),
            ("A* final path", "#388e3c"),
            ("Dijkstra final path", "#1565c0"),
            ("Brute Force final path", "#d32f2f"),
            ("Explored samples", "#2196f3"),
        ]
        y = y0
        for text, color in lines:
            if color:
                r = 7 * self.zoom
                oid = self.canvas.create_oval(x0, y, x0+2*r, y+2*r, fill=color, outline="#424242")
                self.legend_ids.append(oid)
                tid = self.canvas.create_text(x0+2*r+6 * self.zoom, y+r, text=text, anchor="w",
                                             font=("Helvetica", font_size))
                self.legend_ids.append(tid)
                y += 2*r + pad
            else:
                tid = self.canvas.create_text(x0, y, text=text, anchor="w",
                                             font=("Helvetica", bold_font_size, "bold"))
                self.legend_ids.append(tid)
                y += 18 * self.zoom

    def animate_paths(self, paths, color="#2196f3", width=2, delay_ms=250, stop_event: threading.Event = None):
        for it in self.path_items:
            self.canvas.delete(it)
        self.path_items.clear()

        def draw_one(i):
            if stop_event and stop_event.is_set():
                return
            if i >= len(paths):
                return
            p = paths[i]
            for a, b in zip(p, p[1:]):
                ax, ay = self._scaled_coords(self.graph.vertices[a]['x'], self.graph.vertices[a]['y'])
                bx, by = self._scaled_coords(self.graph.vertices[b]['x'], self.graph.vertices[b]['y'])
                mx, my = self._curve_offset_for(a, b)
                mx, my = self._scaled_coords(mx, my)
                lid = self.canvas.create_line(ax, ay, mx, my, bx, by,
                                             smooth=True, width=width * self.zoom, fill=color, stipple="")
                self.path_items.append(lid)
            self.canvas.after(delay_ms, lambda: draw_one(i+1))

        if paths:
            draw_one(0)

    def draw_final_path(self, path, color, width=5):
        if not path or len(path) < 2:
            return
        for a, b in zip(path, path[1:]):
            ax, ay = self._scaled_coords(self.graph.vertices[a]['x'], self.graph.vertices[a]['y'])
            bx, by = self._scaled_coords(self.graph.vertices[b]['x'], self.graph.vertices[b]['y'])
            mx, my = self._curve_offset_for(a, b)
            mx, my = self._scaled_coords(mx, my)
            lid = self.canvas.create_line(ax, ay, mx, my, bx, by,
                                         smooth=True, width=width * self.zoom, fill=color)
            self.path_items.append(lid)


# Tkinter App


class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Navigation Simulator — A* vs Dijkstra vs Brute Force")
        self.root.geometry("1400x900")

        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TButton", padding=6, relief="flat", background="#eeeeee")
        style.configure("TLabel", padding=4)
        style.configure("TCheckbutton", padding=4)
        style.configure("TCombobox", padding=4)

        self.graph = Graph()
        self._populate_default_map()
        self.canvas_frame = ttk.Frame(self.root)
        self.canvas_frame.grid(row=0, column=0, rowspan=20, padx=8, pady=8, sticky="nsew")
        self.canvas = tk.Canvas(self.canvas_frame, width=1000, height=800, bg="#fafafa",
                                highlightthickness=1, highlightbackground="#bdbdbd")
        self.canvas.pack(fill="both", expand=True)
        self.visualizer = Visualizer(self.canvas, self.graph)
        self._center_map()

       
        self.panel_canvas = tk.Canvas(self.root, bg="#fafafa", highlightthickness=0)
        self.panel_scroll = ttk.Scrollbar(self.root, orient="vertical", command=self.panel_canvas.yview)
        self.panel = ttk.Frame(self.panel_canvas, padding=10, relief="raised", borderwidth=1)
        self.panel_canvas.create_window((0, 0), window=self.panel, anchor="nw")

        
        self.panel.bind("<Configure>", self._configure_panel)

       
        self.panel_canvas.configure(yscrollcommand=self.panel_scroll.set)

       
        self.panel_canvas.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        self.panel_scroll.grid(row=0, column=2, sticky="ns", padx=0, pady=8)

        
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)  
        self.root.columnconfigure(2, weight=0) 
        self.root.rowconfigure(0, weight=1)

        self.start_var = tk.StringVar(value="Home")
        self.goal_var = tk.StringVar(value="Airport")
        self.show_samples_var = tk.BooleanVar(value=True)
        self.demo_mode_var = tk.BooleanVar(value=True)
        self.traffic_pct_var = tk.IntVar(value=20)
        self.anim_speed_var = tk.StringVar(value="Normal")
        self.status_text = tk.StringVar(value="Ready.")
        self.metrics_vars = {
            'runtime': tk.StringVar(value="0 ms"),
            'visited': tk.StringVar(value="0"),
            'paths': tk.StringVar(value="0"),
            'cost': tk.StringVar(value="0.0"),
        }

        self.dragging_node = None
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.total_dx = 0
        self.total_dy = 0
        self.pan_last_x = 0
        self.pan_last_y = 0

        self.executor = ThreadPoolExecutor(max_workers=2)
        self.stop_event = threading.Event()
        self.current_future = None

        self._build_controls()
        self._bind_events()
        self.redraw()

        self.graph.randomize_dynamic(self.traffic_pct_var.get())
        self.redraw()

        # Log graph connectivity
        components = self.graph.get_connected_components()
        self.log_msg(f"Graph has {len(components)} connected component(s).")
        if len(components) > 1:
            self.log_msg("Warning: Graph is disconnected. Some paths may not exist.")

        # Bind mouse wheel for scrolling the panel
        self.panel_canvas.bind_all("<MouseWheel>", self._on_mouse_wheel_panel)

    def _center_map(self):
        if not self.graph.vertices:
            return
        avg_x = sum(d['x'] for d in self.graph.vertices.values()) / len(self.graph.vertices)
        avg_y = sum(d['y'] for d in self.graph.vertices.values()) / len(self.graph.vertices)
        self.visualizer.pan_x = avg_x
        self.visualizer.pan_y = avg_y

    def _build_controls(self):
        ttk.Label(self.panel, text="Start:").grid(row=0, column=0, sticky="w")
        self.start_combo = ttk.Combobox(self.panel, textvariable=self.start_var,
                                        values=sorted(self.graph.vertices.keys()), state="readonly", width=26)
        self.start_combo.grid(row=0, column=1, sticky="ew", pady=2)

        ttk.Label(self.panel, text="Goal:").grid(row=1, column=0, sticky="w")
        self.goal_combo = ttk.Combobox(self.panel, textvariable=self.goal_var,
                                       values=sorted(self.graph.vertices.keys()), state="readonly", width=26)
        self.goal_combo.grid(row=1, column=1, sticky="ew", pady=2)

        ttk.Separator(self.panel).grid(row=2, column=0, columnspan=2, sticky="ew", pady=6)

        ttk.Button(self.panel, text="Run A*", command=self.on_run_astar).grid(row=3, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Run Dijkstra", command=self.on_run_dijkstra).grid(row=4, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Run Brute Force (Demo)", command=self.on_run_bruteforce_demo).grid(row=5, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Run Brute Force (Full)", command=self.on_run_bruteforce_full).grid(row=6, column=0, columnspan=2, sticky="ew", pady=2)

        self.small_demo_chk = ttk.Checkbutton(self.panel, text="Small Demo Mode (safe)", variable=self.demo_mode_var)
        self.small_demo_chk.grid(row=7, column=0, columnspan=2, sticky="w")

        ttk.Separator(self.panel).grid(row=8, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Label(self.panel, text="Dynamic Traffic %:").grid(row=9, column=0, sticky="w")
        self.pct_slider = ttk.Scale(self.panel, from_=0, to=60, orient="horizontal", variable=self.traffic_pct_var)
        self.pct_slider.grid(row=9, column=1, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Randomize Traffic", command=self.on_randomize_traffic).grid(row=10, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Apply Dynamic (manual)", command=self.on_set_dynamic).grid(row=11, column=0, columnspan=2, sticky="ew", pady=2)

        ttk.Separator(self.panel).grid(row=12, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Button(self.panel, text="Add Vertex", command=self.on_add_vertex).grid(row=13, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Delete Vertex", command=self.on_delete_vertex).grid(row=14, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Add Edge", command=self.on_add_edge).grid(row=15, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Delete Edge", command=self.on_delete_edge).grid(row=16, column=0, columnspan=2, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Set Fixed", command=self.on_set_fixed).grid(row=17, column=0, columnspan=2, sticky="ew", pady=2)

        ttk.Separator(self.panel).grid(row=18, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Button(self.panel, text="Save Graph", command=self.on_save).grid(row=19, column=0, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Load Graph", command=self.on_load).grid(row=19, column=1, sticky="ew", pady=2)

        ttk.Separator(self.panel).grid(row=20, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Checkbutton(self.panel, text="Show explored 10 sample paths", variable=self.show_samples_var).grid(row=21, column=0, columnspan=2, sticky="w")
        ttk.Label(self.panel, text="Animation Speed:").grid(row=22, column=0, sticky="w")
        ttk.Combobox(self.panel, textvariable=self.anim_speed_var, values=["Fast", "Normal", "Slow"],
                     state="readonly", width=12).grid(row=22, column=1, sticky="e")

        ttk.Separator(self.panel).grid(row=23, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Button(self.panel, text="Zoom In", command=self.zoom_in).grid(row=24, column=0, sticky="ew", pady=2)
        ttk.Button(self.panel, text="Zoom Out", command=self.zoom_out).grid(row=24, column=1, sticky="ew", pady=2)

        ttk.Button(self.panel, text="Clear Paths", command=self.on_clear_paths).grid(row=25, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Button(self.panel, text="Cancel / Stop", command=self.on_cancel).grid(row=26, column=0, columnspan=2, sticky="ew", pady=4)

        ttk.Separator(self.panel).grid(row=27, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Label(self.panel, text="Metrics", font=("Helvetica", 10, "bold")).grid(row=28, column=0, columnspan=2, sticky="w")
        self._metric_row("Runtime:", 'runtime', 29)
        self._metric_row("Visited:", 'visited', 30)
        self._metric_row("Paths Explored:", 'paths', 31)
        self._metric_row("Final Cost:", 'cost', 32)

        ttk.Separator(self.panel).grid(row=33, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Label(self.panel, text="Status / Log").grid(row=34, column=0, columnspan=2, sticky="w")
        self.log = tk.Text(self.panel, width=36, height=12, state="disabled", wrap="word")
        self.log.grid(row=35, column=0, columnspan=2, sticky="nsew", pady=4)
        self.panel.rowconfigure(35, weight=1)

        self._click_state = {'next': 'start'}

    def _metric_row(self, label, key, row):
        ttk.Label(self.panel, text=label).grid(row=row, column=0, sticky="w")
        ttk.Label(self.panel, textvariable=self.metrics_vars[key]).grid(row=row, column=1, sticky="e")

    def _bind_events(self):
        self.canvas.bind("<ButtonPress-1>", self._on_mouse_press)
        self.canvas.bind("<B1-Motion>", self._on_mouse_motion)
        self.canvas.bind("<ButtonRelease-1>", self._on_mouse_release)
        self.canvas.bind("<ButtonPress-3>", self._pan_start)
        self.canvas.bind("<B3-Motion>", self._pan_motion)
        self.canvas.bind("<MouseWheel>", self._on_mouse_wheel)

    def _on_mouse_press(self, event):
        world_x = self.visualizer.pan_x + (event.x - self.visualizer.center_x) / self.visualizer.zoom
        world_y = self.visualizer.pan_y + (event.y - self.visualizer.center_y) / self.visualizer.zoom
        nearest = None
        best = 1e9
        for name, d in self.graph.vertices.items():
            dx = d['x'] - world_x
            dy = d['y'] - world_y
            dist = dx**2 + dy**2
            if dist < best:
                best = dist
                nearest = name
        r = self.visualizer.node_radius + 5
        if nearest and best < r**2:
            self.dragging_node = nearest
            self.drag_start_x = world_x
            self.drag_start_y = world_y
            self.total_dx = 0
            self.total_dy = 0
        else:
            self.dragging_node = None

    def _on_mouse_motion(self, event):
        if self.dragging_node:
            world_x = self.visualizer.pan_x + (event.x - self.visualizer.center_x) / self.visualizer.zoom
            world_y = self.visualizer.pan_y + (event.y - self.visualizer.center_y) / self.visualizer.zoom
            dx = world_x - self.drag_start_x
            dy = world_y - self.drag_start_y
            self.graph.vertices[self.dragging_node]['x'] += dx
            self.graph.vertices[self.dragging_node]['y'] += dy
            self.drag_start_x = world_x
            self.drag_start_y = world_y
            self.total_dx += dx
            self.total_dy += dy
            self.redraw()

    def _on_mouse_release(self, event):
        if self.dragging_node:
            if math.hypot(self.total_dx, self.total_dy) < 0.1:
                if self._click_state['next'] == 'start':
                    self.start_var.set(self.dragging_node)
                    self._click_state['next'] = 'goal'
                else:
                    self.goal_var.set(self.dragging_node)
                    self._click_state['next'] = 'start'
                self.log_msg(f"Selected {self.dragging_node} as {'Start' if self._click_state['next']=='goal' else 'Goal'}.")
                self.redraw()
            self.dragging_node = None

    def _pan_start(self, event):
        self.pan_last_x = event.x
        self.pan_last_y = event.y

    def _pan_motion(self, event):
        dx = (event.x - self.pan_last_x) / self.visualizer.zoom
        dy = (event.y - self.pan_last_y) / self.visualizer.zoom
        self.visualizer.pan_x -= dx
        self.visualizer.pan_y -= dy
        self.pan_last_x = event.x
        self.pan_last_y = event.y
        self.redraw()

    def _on_mouse_wheel(self, event):
        if event.delta > 0:
            self.zoom_in()
        else:
            self.zoom_out()

    def _configure_panel(self, event):
        self.panel_canvas.configure(scrollregion=self.panel_canvas.bbox("all"))

    def _on_mouse_wheel_panel(self, event):
        self.panel_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def zoom_in(self):
        self.visualizer.set_zoom(self.visualizer.zoom * 1.2)
        self.log_msg(f"Zoomed in to {self.visualizer.zoom:.2f}x")

    def zoom_out(self):
        self.visualizer.set_zoom(self.visualizer.zoom / 1.2)
        self.log_msg(f"Zoomed out to {self.visualizer.zoom:.2f}x")

    def on_run_astar(self):
        self._start_task(self._task_astar)

    def on_run_dijkstra(self):
        self._start_task(self._task_dijkstra)

    def on_run_bruteforce_demo(self):
        self.demo_mode_var.set(True)
        self._start_task(self._task_bruteforce)

    def on_run_bruteforce_full(self):
        if not messagebox.askyesno("Confirm Full Brute Force",
                                   "Full brute force can be slow on large graphs.\nProceed?"):
            return
        self.demo_mode_var.set(False)
        self._start_task(self._task_bruteforce)

    def on_randomize_traffic(self):
        pct = self.traffic_pct_var.get()
        self.graph.randomize_dynamic(pct)
        self.log_msg(f"Dynamic traffic applied to ~{pct}% of edges.")
        self.redraw()

    def on_set_dynamic(self):
        u = simpledialog.askstring("Set Dynamic", "Edge FROM:", parent=self.root)
        if not u: return
        v = simpledialog.askstring("Set Dynamic", "Edge TO:", parent=self.root)
        if not v: return
        if u not in self.graph.vertices or v not in self.graph.vertices:
            messagebox.showerror("Error", "Both vertices must exist.")
            return
        try:
            dyn = float(simpledialog.askstring("Set Dynamic", "Dynamic penalty value:", parent=self.root) or "0")
            self.graph.set_dynamic(u, v, dyn)
            self.log_msg(f"Set dynamic penalty {dyn} on edge {u}—{v}.")
            self.redraw()
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_set_fixed(self):
        u = simpledialog.askstring("Set Fixed", "Edge FROM:", parent=self.root)
        if not u: return
        v = simpledialog.askstring("Set Fixed", "Edge TO:", parent=self.root)
        if not v: return
        try:
            val = float(simpledialog.askstring("Set Fixed", "Fixed distance value:", parent=self.root) or "0")
            self.graph.set_fixed(u, v, val)
            self.log_msg(f"Set fixed distance {val} on edge {u}—{v}.")
            self.redraw()
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_add_vertex(self):
        name = simpledialog.askstring("Add Vertex", "Name:", parent=self.root)
        if not name: return
        try:
            x = float(simpledialog.askstring("Add Vertex", "x coordinate (px):", parent=self.root) or "100")
            y = float(simpledialog.askstring("Add Vertex", "y coordinate (px):", parent=self.root) or "100")
            self.graph.add_vertex(name, x, y)
            self._refresh_combos()
            self.redraw()
            self.log_msg(f"Added vertex {name} at ({x:.1f},{y:.1f}).")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_delete_vertex(self):
        name = simpledialog.askstring("Delete Vertex", "Name:", parent=self.root)
        if not name: return
        try:
            self.graph.delete_vertex(name)
            self._refresh_combos()
            self.redraw()
            self.log_msg(f"Deleted vertex {name}.")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_add_edge(self):
        u = simpledialog.askstring("Add Edge", "FROM:", parent=self.root)
        if not u: return
        v = simpledialog.askstring("Add Edge", "TO:", parent=self.root)
        if not v: return
        try:
            fixed_s = simpledialog.askstring("Add Edge", "Fixed distance (blank = Euclidean):", parent=self.root)
            fixed = None if (fixed_s is None or fixed_s.strip() == "") else float(fixed_s)
            self.graph.add_edge(u, v, fixed)
            self.redraw()
            self.log_msg(f"Added edge {u}—{v} (fixed {self.graph.adj[u][v]['fixed']:.1f}).")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_delete_edge(self):
        u = simpledialog.askstring("Delete Edge", "FROM:", parent=self.root)
        if not u: return
        v = simpledialog.askstring("Delete Edge", "TO:", parent=self.root)
        if not v: return
        try:
            self.graph.delete_edge(u, v)
            self.redraw()
            self.log_msg(f"Deleted edge {u}—{v}.")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def on_save(self):
        fn = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")])
        if fn:
            data = {
                'vertices': self.graph.vertices,
                'adj': {k: dict(v) for k, v in self.graph.adj.items()}
            }
            with open(fn, 'w') as f:
                json.dump(data, f)
            self.log_msg(f"Graph saved to {fn}.")

    def on_load(self):
        fn = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if fn:
            with open(fn, 'r') as f:
                data = json.load(f)
            self.graph.vertices = {k: dict(v) for k, v in data['vertices'].items()}
            self.graph.adj = defaultdict(dict, {k: dict(v) for k, v in data['adj'].items()})
            self._refresh_combos()
            self._center_map()
            self.redraw()
            components = self.graph.get_connected_components()
            self.log_msg(f"Graph loaded from {fn}. Has {len(components)} component(s).")
            if len(components) > 1:
                self.log_msg("Warning: Loaded graph is disconnected.")

    def on_clear_paths(self):
        self.stop_event.set()
        self.visualizer.clear()
        self.metrics_vars['runtime'].set("0 ms")
        self.metrics_vars['visited'].set("0")
        self.metrics_vars['paths'].set("0")
        self.metrics_vars['cost'].set("0.0")
        self.redraw()
        self.log_msg("Cleared paths/animations.")

    def on_cancel(self):
        self.stop_event.set()
        self.log_msg("Cancellation requested.")

    def _start_task(self, func):
        self.on_cancel()
        self.stop_event = threading.Event()
        self.visualizer.clear()
        self.metrics_vars['runtime'].set("...")
        self.metrics_vars['visited'].set("...")
        self.metrics_vars['paths'].set("...")
        self.metrics_vars['cost'].set("...")
        self.current_future = self.executor.submit(func, self.stop_event)

    def _task_astar(self, stop_event: threading.Event):
        start = self.start_var.get()
        goal = self.goal_var.get()
        if start not in self.graph.vertices or goal not in self.graph.vertices:
            self._after(lambda: messagebox.showerror("Error", "Start/Goal must exist."))
            return
        path, cost, runtime, visited, sample = astar(self.graph, start, goal, stop_event, self.log_msg)
        def ui():
            self.redraw()
            self.metrics_vars['runtime'].set(f"{runtime:.1f} ms")
            self.metrics_vars['visited'].set(str(len(visited)))
            self.metrics_vars['paths'].set(str(len(sample)))
            self.metrics_vars['cost'].set(f"{cost:.1f}" if math.isfinite(cost) else "inf")
            self.log_msg(f"A* visited {len(visited)} nodes in {runtime:.1f} ms. Cost = {cost:.1f}" if path else "A* did not find a path.")
            if self.show_samples_var.get() and sample:
                delay = {"Fast": 80, "Normal": 220, "Slow": 420}[self.anim_speed_var.get()]
                self.visualizer.animate_paths(sample, color="#2196f3", width=3, delay_ms=delay, stop_event=stop_event)
            if path:
                self.visualizer.draw_final_path(path, color="#388e3c", width=6)
        self._after(ui)

    def _task_dijkstra(self, stop_event: threading.Event):
        start = self.start_var.get()
        goal = self.goal_var.get()
        if start not in self.graph.vertices or goal not in self.graph.vertices:
            self._after(lambda: messagebox.showerror("Error", "Start/Goal must exist."))
            return
        path, cost, runtime, visited, sample = dijkstra(self.graph, start, goal, stop_event, self.log_msg)
        def ui():
            self.redraw()
            self.metrics_vars['runtime'].set(f"{runtime:.1f} ms")
            self.metrics_vars['visited'].set(str(len(visited)))
            self.metrics_vars['paths'].set(str(len(sample)))
            self.metrics_vars['cost'].set(f"{cost:.1f}" if math.isfinite(cost) else "inf")
            self.log_msg(f"Dijkstra visited {len(visited)} nodes in {runtime:.1f} ms. Cost = {cost:.1f}" if path else "Dijkstra did not find a path.")
            if self.show_samples_var.get() and sample:
                delay = {"Fast": 80, "Normal": 220, "Slow": 420}[self.anim_speed_var.get()]
                self.visualizer.animate_paths(sample, color="#2196f3", width=3, delay_ms=delay, stop_event=stop_event)
            if path:
                self.visualizer.draw_final_path(path, color="#1565c0", width=6)
        self._after(ui)

    def _nearest_demo_nodes(self, start, goal, k_each=10):
        names = list(self.graph.vertices.keys())
        if len(names) <= k_each*2:
            return names
        def k_near(src):
            dists = [(self.graph.euclidean(src, n), n) for n in names if n != src]
            dists.sort()
            return [src] + [n for _, n in dists[:k_each]]
        return list(set(k_near(start) + k_near(goal)))

    def _task_bruteforce(self, stop_event: threading.Event):
        start = self.start_var.get()
        goal = self.goal_var.get()
        if start not in self.graph.vertices or goal not in self.graph.vertices:
            self._after(lambda: messagebox.showerror("Error", "Start/Goal must exist."))
            return
        demo_nodes = self._nearest_demo_nodes(start, goal, k_each=8) if self.demo_mode_var.get() else None
        cap = 10000 if demo_nodes else 100000
        path, cost, runtime, explored, sample = brute_force_all_paths(self.graph, start, goal, cap=cap,
                                                                     demo_nodes=demo_nodes, stop_event=stop_event, log_fn=self.log_msg)
        def ui():
            self.redraw()
            self.metrics_vars['runtime'].set(f"{runtime:.1f} ms")
            self.metrics_vars['visited'].set("—")
            self.metrics_vars['paths'].set(str(explored))
            self.metrics_vars['cost'].set(f"{cost:.1f}" if math.isfinite(cost) else "inf")
            scope = "Demo" if demo_nodes else "Full"
            self.log_msg(f"Brute Force ({scope}) explored {explored} paths in {runtime:.1f} ms. Cost = {cost:.1f}" if path else f"Brute Force ({scope}) found no path.")
            if self.show_samples_var.get() and sample:
                delay = {"Fast": 80, "Normal": 220, "Slow": 420}[self.anim_speed_var.get()]
                self.visualizer.animate_paths(sample, color="#ffd54f", width=3, delay_ms=delay, stop_event=stop_event)
            if path:
                self.visualizer.draw_final_path(path, color="#d32f2f", width=6)
        self._after(ui)

    def _after(self, fn):
        self.root.after(0, fn)

    def redraw(self):
        self.visualizer.redraw_all(start=self.start_var.get(), goal=self.goal_var.get())

    def log_msg(self, msg):
        self.status_text.set(msg)
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _refresh_combos(self):
        names = sorted(self.graph.vertices.keys())
        self.start_combo.configure(values=names)
        self.goal_combo.configure(values=names)

    def _populate_default_map(self):
        nodes = {
            "Airport": (820, 120), "Central Park": (420, 160), "University": (520, 210),
            "City Hospital": (650, 240), "Main Mall": (360, 260), "Stadium": (700, 160),
            "Museum": (540, 120), "Library": (480, 300), "Old Town": (280, 320),
            "Tech Hub": (600, 300), "Bus Terminal": (440, 360), "Grand Hotel": (760, 260),
            "Riverside": (220, 420), "West Gate": (160, 360), "East Gate": (880, 300),
            "North Square": (420, 80), "South Square": (420, 520), "Harbor": (780, 420),
            "Industrial Park": (680, 380), "City Hall": (500, 360), "Police HQ": (560, 340),
            "Fire Station": (620, 360), "Botanical Garden": (340, 160), "Zoo": (300, 220),
            "Aquarium": (740, 320), "Convention Center": (640, 140), "Art District": (260, 260),
            "Financial Center": (560, 220), "Tech Park": (620, 200), "Sports Complex": (700, 280),
            "Suburb Heights": (160, 520), "Lakeview": (280, 540), "Valley Point": (360, 560),
            "Hillcrest": (480, 560), "Green Meadows": (600, 540), "River Bend": (720, 520),
            "Seaside Pier": (860, 460), "Railway Station": (520, 400), "Market Street": (380, 380),
            "Cinema Plaza": (340, 320), "Tech Incubator": (580, 260), "Startup Lane": (620, 260),
            "Innovation Lab": (580, 320), "Community Center": (420, 440), "Home": (240, 360),
            "Office": (740, 220), "City Theater": (460, 220), "South Park": (420, 600),
            "East Market": (820, 340), "North Park": (420, 20), "West Market": (100, 300),
            "River Docks": (760, 500), "Tech Plaza": (560, 280), "Bike Hub": (300, 420),
            "Food Court": (380, 300), "Art Museum": (500, 160), "Science Center": (580, 180),
            "Medical College": (620, 320), "City Gardens": (320, 120), "Heritage Site": (200, 300),
        }




# adding nodes


        for name, (x, y) in nodes.items():
            self.graph.add_vertex(name, x, y)

        self.graph.connect_k_nearest(k=3)

        try:
            self.graph.add_edge("Home", "Central Park", fixed=5.0)
            self.graph.add_edge("Home", "Main Mall", fixed=7.0)
            self.graph.set_dynamic("Home", "Central Park", 10.0)
            self.graph.set_dynamic("Home", "Main Mall", 0.0)
        except Exception:
            if "Central Park" in self.graph.adj.get("Home", {}):
                self.graph.set_fixed("Home", "Central Park", 5.0)
                self.graph.set_dynamic("Home", "Central Park", 10.0)
            if "Main Mall" in self.graph.adj.get("Home", {}):
                self.graph.set_fixed("Home", "Main Mall", 7.0)
                self.graph.set_dynamic("Home", "Main Mall", 0.0)

        components = self.graph.get_connected_components()
        home_comp = None
        airport_comp = None
        for comp in components:
            if "Home" in comp:
                home_comp = comp
            if "Airport" in comp:
                airport_comp = comp
        if home_comp and airport_comp and home_comp != airport_comp:
            self.graph.add_edge("Home", "Airport", fixed=self.graph.euclidean("Home", "Airport"))
            self.log_msg("Added Home—Airport edge to ensure connectivity. [08:40 AM +0545, Monday, September 01, 2025]")

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    app = App()
    app.run()