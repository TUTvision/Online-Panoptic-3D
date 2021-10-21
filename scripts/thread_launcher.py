
import math
import threading

class Thread(threading.Thread):
   def __init__(self, callback, args, kwargs):
      threading.Thread.__init__(self)

      self.callback = callback
      self.args = args
      self.kwargs = kwargs

   def run(self):
       self.callback(self.args, **self.kwargs)

class Thread_launcher():
    def __init__(self,
        callback,
        n_threads = 8,
        **callback_args):

        self.n_threads = n_threads
        self.callback = callback
        self.callback_args = callback_args
        self.threads = []

    def launch(self, scenes):
        n_scenes = len(scenes)
        partition_size = int(math.ceil(n_scenes/self.n_threads))

        last_thread = False

        idx = 0
        for i in range(self.n_threads):
            if idx + partition_size >= n_scenes:
                partition = scenes[idx:]

                last_thread = True
            else:
                partition = scenes[idx : idx + partition_size]

            idx += partition_size

            t = Thread(self.callback, partition, self.callback_args)
            t.start()
            self.threads.append(t)

            if last_thread: break

    def join(self):
        for t in self.threads:
            t.join()

        self.threads.clear()
