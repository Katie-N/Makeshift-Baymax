import threading
import hello
import hello2

thread1 = threading.Thread(target=hello.main)
thread2 = threading.Thread(target=hello2.main)

thread1.start()
thread2.start()

thread1.join()
thread2.join()