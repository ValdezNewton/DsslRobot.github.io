---
slug: thread_in_ros2
title: An Intro to Thread In ROS2
authors: [CabbageDog]
tags: [hello]
---

# Thread In ROS2 ---- Examples in Python

This blog aims to provide some basic knowledge about threading in ROS2.
<!-- truncate -->

## Basic Concepts

1. **Concurrency/Parallelism**: 
   - Concurrency is when two or more tasks can start, run, and complete in overlapping time periods. It doesn't necessarily mean they'll ever both be running at the same instant. For example, multitasking on a single-core machine.
   - Parallelism is when tasks literally run at the same time, e.g., on a multicore processor. 

    You can see more examples on the Internet.

2. **Threading**:
   Threading is a concurrent execution model whereby multiple threads take turns executing tasks. One process can contain multiple threads.

3. **Multiprocessing/Multithreading**:
    Multiprocessing is a means to effect parallelism, and it entails spreading tasks over a computer’s central processing units (CPUs, or cores). Its effect is bound to CPU power(number of core, etc), so multiprocessing is suitable for CPU-bound task. A CPU-bound task is characterized by the computer’s cores continually working hard from start to finish.

    Multithreading is used to optimize the efficiency of a process. It is suitable for IO-bound task, which is dominated by a lot of waiting on input/output to complete.

4. **Asynchronous**
    Asynchronous doesn't have a rigorous definition. Bascially we think:
    - Asynchronous routines are able to “pause” while waiting on their ultimate result and let other routines run in the meantime.
    - Asynchronous code gives the look and feel of concurrency.


## `Asyncio` Pakcage

`Asyncio` Pakcage is a Python official package to perform **cooperative multitasking**, which uses one single thread and one process to "impelment"  concurrency. `rclpy` also uses `asyncio` for many core functions such as `spin()`.

### Coroutines
A coroutine is a function that can suspend its execution before reaching return, and it can indirectly pass control to another coroutine for some time.

[Here](https://github.com/glaciercoder/ros2_thread_demo/blob/main/src/ros2_thread_demo/ros2_thread_demo/hello_world_sync.py) is a simple coroutine demo, you can compare with its [sync version](https://github.com/glaciercoder/ros2_thread_demo/blob/main/src/ros2_thread_demo/ros2_thread_demo/hello_world_sync.py). `async def` defines a coroutine, in which `await asyncio.sleep(1)` doesn't block the `main()`.

Basically we use grammar:
```py
async def g():
    # Pause here and come back to g() when f() is ready, and the program can do something else now
    r = await f()
    return r
```
[This demo](https://github.com/glaciercoder/ros2_thread_demo/blob/main/src/ros2_thread_demo/ros2_thread_demo/rand.py) is another intresing example for coroutines.

When a coroutine needs the from another coroutine, it will wait for that coroutine to be finished, which is used to chain coroutines, see [this example](https://github.com/glaciercoder/ros2_thread_demo/blob/main/src/ros2_thread_demo/ros2_thread_demo/chained.py).

## `Threading` Package

`Treading` is also a Python official package to impelment concurrency. It literally creates new threads and manage them, which is also useful in multi-thread ROS2 application.

### Create a new thread

One typical usage is to put `spin()` function into a new thread, which makes you implement other blocking functions while not break the execution of callback. I think this [plot](https://github.com/timdodge54/matplotlib_ros_tutorials) function is a good example for this kind of usage.

### `Event`

> *This is one of the simplest mechanisms for communication between threads: one thread signals an event and other threads wait for it.*

The `threading.Event` provides an easy way to share a boolean variable between threads that can act as a trigger for an action. This object is used in `rclpy` official implementation of action client, service client, etc.

```py
# create an instance of an event
event = threading.Event()
# check if the event is set
if event.is_set():
	# do something...
# set the event
event.set()
# mark the event as not set
event.clear()
# wait for the event to be set
event.wait()
```

The important thing is `event.wait()` here. Calling this function will **block** the current thread until the event is marked as set (e.g. another thread calling the `set()` function). 

##  Synchronous vs. asynchronous in `rclpy`

There are two versions of call for service client: `call()` amd `call_async()`, also `send_gaol()` and `send_goal_async()` for action client. Let's see service `call()` as an example, and `send_goal()` is similar.

We can see the following code in [`rclpy/client.py`](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/client.py)
```python
def call_async(self, request: SrvTypeRequest) -> Future:
        """
        Make a service request and asyncronously get the result.

        :param request: The service request.
        :return: A future that completes when the request does.
        :raises: TypeError if the type of the passed request isn't an instance
          of the Request type of the provided service when the client was
          constructed.
        """
        if not isinstance(request, self.srv_type.Request):
            raise TypeError()

        with self._lock:
            with self.handle:
                sequence_number = self.__client.send_request(request)
            if sequence_number in self._pending_requests:
                raise RuntimeError(f'Sequence ({sequence_number}) conflicts with pending request')

            future = Future()
            self._pending_requests[sequence_number] = future

            future.add_done_callback(self.remove_pending_request)

        return future
```
We can see that `call_async()` actually calls `send_request()`, which is a python wrapper for `rcl_send_request()` and is called by `rcl`. `rcl_send_request()` is non-blocking, see [rcl API]([rcl_send_request](https://docs.ros.org/en/humble/p/rcl/generated/function_client_8h_1a238e531f67dbdcc69a7f85f260b25212.html#client_8h_1a238e531f67dbdcc69a7f85f260b25212)). After this non-blocking call, `call_async()` creates a future for this call and put is in the `pending_request`, then add `remove_pending_request` as the done callback for this future. This done callback will be executed by the executor which the client node is bound to and adding this callback will not block anything.

So, `call_async()` is a totally non-blocking function and you can use this freely in either another callback or other thread you want. The only thing you should care about is this a future can add not only one callback. If you want to add other callbacks to the future returned by `call_async()`, you need to make sure that callback can be executed properly and doesn't block. Anyway, that's irrelavant to `call_async()`.

Now let's take a look at its synchronous version: `call()`:
```python
def call(
        self,
        request: SrvRequestT,
        timeout_sec: Optional[float] = None
    ) -> Optional[SrvResponseT]:
        """
        Make a service request and wait for the result.

        .. warning:: Do not call this method in a callback, or a deadlock or timeout may occur.

        :param request: The service request.
        :param timeout_sec: Seconds to wait. If ``None``, then wait forever.
        :return: The service response, or None if timed out.
        :raises: TypeError if the type of the passed request isn't an instance
          of the Request type of the provided service when the client was
          constructed.
        """
        if not isinstance(request, self.srv_type.Request):
            raise TypeError()

        event = threading.Event()

        def unblock(future: Future[SrvResponseT]) -> None:
            nonlocal event
            event.set()

        future = self.call_async(request)
        future.add_done_callback(unblock)

        # Check future.done() before waiting on the event.
        # The callback might have been added after the future is completed,
        # resulting in the event never being set.
        if not future.done():
            if not event.wait(timeout_sec):
                # Timed out. remove_pending_request() to free resources
                self.remove_pending_request(future)

        exception = future.exception()
        if exception is not None:
            raise exception
        return future.result()
```

We can see that `call()` is actually calling `call_aysnc()`. After that , it adds another done callback to it, so far so good. The main modification is that it then uses `event()` to wait for future to be done. We know that `wait()` will block the thread and wait for another thread to set the event. So `call()` is a blocking API. If it just blocks itself, then that's just fine. The problem is, if you put a `call()` in a callback function(Imagine you need another signal from another node and you trigger this call() in the callback), this will block the callback function where it is called. Since the default callbackgroup in ROS2 is `MutuallyExclusiveCallbackGroup()`, which means it execute callback one by one, all other callbacks(including the done callback) will be waiting for this blocked callback, and the `call()` itself is waiting for other callbacks to "save it". This a typical **deadlock**.

One way to solve this deadlock is to put `spin()` in another thread, and make `call()` outside of the thread where `spin()` is in. While `call()` blocks, other callbacks can be executed properly. Check [this code](https://github.com/glaciercoder/ros2_thread_demo/blob/main/src/ros2_thread_demo/ros2_thread_demo/service_sync_call_threads.py). You may think we can use `ReentrantCallbackGroup` to solve this problem, but sadly we can't. The reason you can see in the above code: if you put `spin()` after `call()`, callbacks will not be executed since you need callback to set event first before you can get through `call()`. If you put `spin()` before `call()`, `spin()` itself will block the `call()`.

The above implementation can be used when you just import the client class and use `send_request()` in your code. This is convenient once you put spin in a dedicated thread. But what if you really want to control the send via another signal? Then you have to put `call()` into a callback. For this, ROS2 official suggested a way to make this possible by using callbackgroup, see [this code](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html). However, this is actually **impossible**. The problem is: although the executor **can** tackle with multiple callbackgroups, it do this by `asyncio` rather than true multiprocessing for `SingleThreadedExecutor`. Which means, if you don't use a coroutine for blocking, like `call()`, the whole thread will be blocked and `spin()` can not execute other callbacks at all!! ----Even if the callback from the different callback group!!! 

To truely solve this problem, the only way is to use a multithreaded executor, you can check [my example](https://github.com/glaciercoder/ros2_thread_demo/blob/main/src/ros2_thread_demo/ros2_thread_demo/service_sync_call_deadlock.py) here to see how does this work. 

## `spin()`, `spin_once()` and `spin_once_until_future_complete()`
The `spin()` function and its variants in `executor` class is the most important function in ROS2. Let's see how it is implemented([source code](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/executors.py)).

In the base class `Executor`, `spin()` and `spin_until_future_complete()` is defined
```py
def spin(self) -> None:
        """Execute callbacks until shutdown."""
        while self._context.ok() and not self._is_shutdown:
            self.spin_once()

def spin_until_future_complete(self, future: Future, timeout_sec: float = None) -> None:
        """Execute callbacks until a given future is done or a timeout occurs."""
        # Make sure the future wakes this executor when it is done
        future.add_done_callback(lambda x: self.wake())

        if timeout_sec is None or timeout_sec < 0:
            while self._context.ok() and not future.done() and not self._is_shutdown:
                self.spin_once_until_future_complete(future, timeout_sec)
        else:
            start = time.monotonic()
            end = start + timeout_sec
            timeout_left = TimeoutObject(timeout_sec)

            while self._context.ok() and not future.done() and not self._is_shutdown:
                self.spin_once_until_future_complete(future, timeout_left)
                now = time.monotonic()

                if now >= end:
                    return

                timeout_left.timeout = end - now
```
We can see that both `spin()` and `spin_until_future_complete()` will block the code. But they are using functions like `spin_once()` and `spin_until_future_complete()`, which means it will not cause deadlock. We can see that `spin_once()` and `spin_until_future_complete()` is not implemented in base class, and it is left to be implemented in the subclass. Let's see the implementation in `SingleThreadedExecutor`:

```py
def spin_once(self, timeout_sec: float = None) -> None:
        self._spin_once_impl(timeout_sec)

def spin_once_until_future_complete(
        self,
        future: Future,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None
    ) -> None:
        self._spin_once_impl(timeout_sec)
```
Wow~~, so we see that `spin_once()` and `spin_once_until_future_complete()` is actually the same function. So acutally, all variants of `spin()` is calling `_spin_once_impl`. How this is implemented? See:
```py
def _spin_once_impl(
        self,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
        wait_condition: Callable[[], bool] = lambda: False
    ) -> None:
        try:
            handler, entity, node = self.wait_for_ready_callbacks(
                timeout_sec, None, wait_condition)
        except ...
        else:
            self._executor.submit(handler)
            self._futures.append(handler)
            for future in self._futures:  # check for any exceptions
                if future.done():
                    self._futures.remove(future)
                    future.result()
```

We remove the exception check in code, so what `_spin_once_impl` does is that it get executable callbakcs from the list, and then call its handler, then clear some finished futures. Let's see first part first.

### `wait_for_ready_callbacks()`
`wait_for_ready_callbacks()` calls a hidden function `_wait_for_ready_callbacks`, we take down the important part here:
```py
    def _wait_for_ready_callbacks(
        self,
        timeout_sec: Optional[Union[float, TimeoutObject]] = None,
        nodes: List['Node'] = None,
        condition: Callable[[], bool] = lambda: False,
    ) -> Generator[Tuple[Task, WaitableEntityType, 'Node'], None, None]:
        """
        Yield callbacks that are ready to be executed.

        :raise TimeoutException: on timeout.
        :raise ShutdownException: on if executor was shut down.

        :param timeout_sec: Seconds to wait. Block forever if ``None`` or negative.
            Don't wait if 0.
        :param nodes: A list of nodes to wait on. Wait on all nodes if ``None``.
        :param condition: A callable that makes the function return immediately when it evaluates
            to True.
        """
        timeout_timer = None
        ... # Time out settings

        yielded_work = False
        while not yielded_work and not self._is_shutdown and not condition():
            # Refresh "all" nodes in case executor was woken by a node being added or removed
            nodes_to_use = nodes
            if nodes is None:
                nodes_to_use = self.get_nodes()

            # Yield tasks in-progress before waiting for new work
            tasks = None
            with self._tasks_lock:
                tasks = list(self._tasks)
            if tasks:
                for task, entity, node in reversed(tasks):
                    if (not task.executing() and not task.done() and
                            (node is None or node in nodes_to_use)):
                        yielded_work = True
                        yield task, entity, node
                with self._tasks_lock:
                    # Get rid of any tasks that are done
                    self._tasks = list(filter(lambda t_e_n: not t_e_n[0].done(), self._tasks))

            # Gather entities that can be waited on
            subscriptions: List[Subscription] = []
            timers: List[Timer] = []
            clients: List[Client] = []
            services: List[Service] = []
            guards: List[GuardCondition] = []
            waitables: List[Waitable] = []
            for node in nodes_to_use:
                subscriptions.extend(filter(self.can_execute, node.subscriptions))
                ... # add entities to the preceding list
                # retrigger a guard condition that was triggered but not handled
                ... # add guards
                ...
            # Construct a wait set
            wait_set = None
            with ExitStack() as context_stack:
                sub_handles = []
                for sub in subscriptions:
                    try:
                        context_stack.enter_context(sub.handle)
                        sub_handles.append(sub.handle)
                    except InvalidHandle:
                        entity_count.num_subscriptions -= 1

                ... # Dispose other handles like client_handles, etc.

                context_stack.enter_context(self._context.handle)

                wait_set = _rclpy.WaitSet(
                    entity_count.num_subscriptions,
                    ... # others
                    self._context.handle)

                wait_set.clear_entities()
                for sub_handle in sub_handles:
                    wait_set.add_subscription(sub_handle)
                ... # other handles

                # Wait for something to become ready
                wait_set.wait(timeout_nsec)
                if self._is_shutdown:
                    raise ShutdownException()
                if not self._context.ok():
                    raise ExternalShutdownException()

                # get ready entities
                subs_ready = wait_set.get_ready_entities('subscription')
                ... # others

                # Mark all guards as triggered before yielding since they're auto-taken
                for gc in guards:
                    if gc.handle.pointer in guards_ready:
                        gc._executor_triggered = True

                # Check waitables before wait set is destroyed
                for node in nodes_to_use:
                    for wt in node.waitables:
                        # Only check waitables that were added to the wait set
                        if wt in waitables and wt.is_ready(wait_set):
                            if wt.callback_group.can_execute(wt):
                                handler = self._make_handler(
                                    wt, node, lambda e: e.take_data(), self._execute_waitable)
                                yielded_work = True
                                yield handler, wt, node

            # Process ready entities one node at a time
            for node in nodes_to_use:
                for tmr in node.timers:
                    if tmr.handle.pointer in timers_ready:
                        # Check timer is ready to workaround rcl issue with cancelled timers
                        if tmr.handle.is_timer_ready():
                            if tmr.callback_group.can_execute(tmr):
                                handler = self._make_handler(
                                    tmr, node, self._take_timer, self._execute_timer)
                                yielded_work = True
                                yield handler, tmr, node

                ... # Make other handlers

            # Check timeout timer
            if (
                timeout_nsec == 0 or
                (timeout_timer is not None and timeout_timer.handle.pointer in timers_ready)
            ):
                raise TimeoutException()
        if self._is_shutdown:
            raise ShutdownException()
        if condition():
            raise ConditionReachedException()
```
The main mechnism in `_wait_for_ready_callbacks` is `WaitSet`. `WaitSet` is a low level component of ROS2, it checks conditions that makes a callback ready. For example, a subscription callback becomes ready when a new message is received on the subscribed topic. `WaitSet` is a **synchronous** implementations which means it will block if no callback is ready, and it is not a queue following FIFO. It just examines callbacks and mark them as ready. Once a callback is ready, it will be added to the `subs_ready`, `_wait_for_ready_callbacks` loops over all callbacks, once it finds a ready callback, it returns instantly.

From the above code we can also see `callback_group` mechanism. If a callback is in a callback group that is not marked as ready(A callback in that callbackgroup is not finished), this callback **will not be called at all!**, no matter what content is in this callback.

### `handler()`

We can see that once a callback is ready to be executed, it will be made into a handler via `_make_handler`:
```py
def _make_handler(
        self,
        entity: WaitableEntityType,
        node: 'Node',
        take_from_wait_list: Callable,
        call_coroutine: Coroutine
    ) -> Task:
        """
        Make a handler that performs work on an entity.

        :param entity: An entity to wait on.
        :param node: The node associated with the entity.
        :param take_from_wait_list: Makes the entity to stop appearing in the wait list.
        :param call_coroutine: Does the work the entity is ready for
        """
        # Mark this so it doesn't get added back to the wait list
        entity._executor_event = True

        async def handler(entity, gc, is_shutdown, work_tracker):
            if is_shutdown or not entity.callback_group.beginning_execution(entity):
                # Didn't get the callback, or the executor has been ordered to stop
                entity._executor_event = False
                gc.trigger()
                return
            with work_tracker:
                arg = take_from_wait_list(entity)

                # Signal that this has been 'taken' and can be added back to the wait list
                entity._executor_event = False
                gc.trigger()

                try:
                    await call_coroutine(entity, arg)
                finally:
                    entity.callback_group.ending_execution(entity)
                    # Signal that work has been done so the next callback in a mutually exclusive
                    # callback group can get executed
                    gc.trigger()
        task = Task(
            handler, (entity, self._guard, self._is_shutdown, self._work_tracker),
            executor=self)
        with self._tasks_lock:
            self._tasks.append((task, entity, node))
        return task
```

What `_make_handler` do is it create a `Task` object and bind a `handler` to it. `handler` is a **coroutine** here. In [`rclpy/task.py`](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/task.py), we can see the following code:
```py
def __call__(self):
        """
        Run or resume a task.

        This attempts to execute a handler. If the handler is a coroutine it will attempt to
        await it. If there are done callbacks it will schedule them with the executor.

        The return value of the handler is stored as the task result.
        """
        if self._done or self._executing or not self._task_lock.acquire(blocking=False):
            return
        try:
            if self._done:
                return
            self._executing = True

            if inspect.iscoroutine(self._handler):
                # Execute a coroutine
                try:
                    self._handler.send(None)
                except StopIteration as e:
                    # The coroutine finished; store the result
                    self._handler.close()
                    self.set_result(e.value)
                    self._complete_task()
                except Exception as e:
                    self.set_exception(e)
                    self._complete_task()
            else:
                # Execute a normal function
                try:
                    self.set_result(self._handler(*self._args, **self._kwargs))
                except Exception as e:
                    self.set_exception(e)
                self._complete_task()

            self._executing = False
        finally:
            self._task_lock.release()
```
The call of task itself is managed by lower level of ROS2 and that is beyond our scope. What is meaningful here is that `spin_once()` acutally **doesn't execute** any callback. It just "dispatch" them. This can cause some confusion, see [this issue](https://github.com/ros2/rclpy/issues/585)

### Write `spin()` in a callback

Sometimes you may want to use `spin_until_future_complete()` in a callback(Say, you are waiting for a future to do other parts of the callback). Is this possible? For `SingleThreadedExecutor`, it processes callback one by one. If you want to execute other callbackes in the callback you are calling `spin_until_future_complete()`, you must put other callbacks in a different callback group.

However, is it enough? The answer is no. See [this example](https://github.com/glaciercoder/ros2_thread_demo/blob/main/src/ros2_thread_demo/ros2_thread_demo/spin_in_callback_deadlock.py). When you run this, you will find that the first time `_timer_cb` is executed properly. However, it is triggered no more after that. This may due to the underlying `awake` mechanism of ROS2. So even it may work sometime for putting  `spin_until_future_complete()` in a callback. We don't recommend so. The solution to this is simple. You can add a done callback to the future, and set a flag for it. Check [this example](https://github.com/glaciercoder/ros2_thread_demo/blob/main/src/ros2_thread_demo/ros2_thread_demo/spin_in_callback_sol1.py). Also, you can use a coroutine directly as callback and then use `await` into it to wait for future to be done aysnchronously. This can be done since the compatiblity of ROS2 callback and coroutine, see [this example](https://github.com/glaciercoder/ros2_thread_demo/blob/main/src/ros2_thread_demo/ros2_thread_demo/spin_in_callback_sol2.py). You can check [this repo](https://github.com/m2-farzan/ros2-asyncio) for advanced usage for coroutine in ROS2

## Callback Group
[ROS2 official](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html) has provided a detailed explanation about callback groups. However, the example in the page is not right as we mentioned before. The implementation of callback group is rather simple, see [official code](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/callback_groups.py):
```py
class ReentrantCallbackGroup(CallbackGroup):
    """Allow callbacks to be executed in parallel without restriction."""

    def can_execute(self, entity):
        return True

    def beginning_execution(self, entity):
        return True

    def ending_execution(self, entity):
        pass
```
Once a callback is ready, `ReentrantCallbackGroup` will always return `True` for `can_execute` and the callback will be dispatched.
```py
class MutuallyExclusiveCallbackGroup(CallbackGroup):
    """Allow only one callback to be executing at a time."""

    def __init__(self):
        super().__init__()
        self._active_entity = None
        self._lock = Lock()

    def can_execute(self, entity):
        with self._lock:
            assert weakref.ref(entity) in self.entities
            return self._active_entity is None

    def beginning_execution(self, entity):
        with self._lock:
            assert weakref.ref(entity) in self.entities
            if self._active_entity is None:
                self._active_entity = entity
                return True
        return False

    def ending_execution(self, entity):
        with self._lock:
            assert self._active_entity == entity
            self._active_entity = None
```
Instead, `MutuallyExclusiveCallbackGroup` checks whether there is any member in this group is active. If any, it will not allow any other callback to be executed. The active state is set to False **only if the callback finished**. That means even if your callback is designed to be a coroutine, it still blocks other callbacks in the same `MutuallyExclusiveCallbackGroup`.

If a deadlock is caused by blocking callback(such as your done-callback and the callback which call done-callback in the same group), the deadlock can be solved by using more callbackgroups. If a deadlock is caused by blocking `spin()` or the thread where other callbacks are, adding callbackgroups will do nothing. Use `MultiThreadedExecutor` to avoid this.
