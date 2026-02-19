import numpy as np  # Make numpy available using np.
from matplotlib import pyplot as plt

class graph:
  def __init__(self, n_agents, list_links, undirected=True):
    self.n=n_agents # number of agents. Indices are 0, 1, ..., n-1
    self.graph_E = np.zeros((self.n,self.n))
    self.graph_D = np.zeros((self.n,self.n))
    self.graph_L = np.zeros((self.n,self.n))
    self.graph_W = np.zeros((self.n,self.n))
    #links
    self.list_links = list_links
    self.is_undirected = undirected

    # Adjacency matrix (E) setup
    for indx in range(self.list_links.shape[0]):
      node_i = self.list_links[indx,0]
      node_j = self.list_links[indx,1]
      # print("indx:", indx)
      # print("link element: (", node_i, ",", node_j, ")")
      self.graph_E[node_i, node_j]=1
      if self.is_undirected:
          self.graph_E[node_j, node_i]=1
    # print("Adjacency matrix: ", self.graph_E)

    # Degree matrix (D) setup
    self.graph_D = np.diag(self.graph_E.sum(axis=0))
    # print("Degree matrix: ", Degree_matrix)

    # Laplacian matrix (L) setup
    self.graph_L =  self.graph_D - self.graph_E
    # print("Laplacian matrix: ", Laplacian_matrix)

  def define_W_matrix(self, alpha):
    L_shape = self.graph_L.shape
    self.graph_W = np.eye(L_shape[0], L_shape[1]) - alpha * self.graph_L
    # print("Weight_matrix =", self.graph_W)

  def eigenvals(self):
    # L matrix eigenvals
    l_eigvals = np.linalg.eigvals(self.graph_L)
    print("Laplacian eigen values: ", l_eigvals)

    # Algebraic connectivity
    number = l_eigvals[l_eigvals > 1e-4]
    number_sorted = np.sort(number)
    algebraic_conn = number_sorted[0]
    print("algebraic connectivity: ", algebraic_conn)

    # W matrix eigenvals
    # The closer to one, the slower / Larger than 1 == divergent
    perr_eigvals = np.linalg.eigvals(self.graph_W)
    print("eigen values: ", perr_eigvals)

  def plot_graph(self):
    n=self.graph_E.shape[0]
    v_angles=np.linspace(0, 2*np.pi, n, endpoint=False)
    v_x=np.cos(v_angles)
    v_y=np.sin(v_angles)
    v_x_text=np.cos(v_angles)*1.1
    v_y_text=np.sin(v_angles)*1.1
    # print(v_x)
    # print(v_y)
    plt.title("Graph: links")
    plt.xlabel("x axis")
    plt.ylabel("y axis")

    for i in range(n):
      for j in range(n):
        if self.graph_E[i,j]>0:
          x_ini=v_x[i]
          y_ini=v_y[i]
          dx= v_x[j] -x_ini
          dy= v_y[j] -y_ini
          plt.arrow(x_ini, y_ini, dx, dy, head_length=0.1,length_includes_head=True, head_width=0.05)
    plt.plot(v_x,v_y, "or") ,#without lines, only the dots
    for i in range(n):
      plt.text(v_x_text[i], v_y_text[i], str(i))
    plt.show()

  def select_random_link(self):
    idx = np.random.randint(self.list_links.shape[0])
    i = self.list_links[idx,0]
    j = self.list_links[idx,1]
    return i,j