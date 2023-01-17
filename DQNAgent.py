#function: Defining all possible combination of 3 ranges
def combs(range_size):
    result=[]
    for x in range(1,range_size[0]+1):
        for y in range(1,range_size[1]+1):
            for z in range(1,range_size[2]+1):
                combination = (x,y,z)
                result.append(combination)
    return result

#function: Getting an action, and giving the index number of that action in all possible combinations obtained previously from combs
def reverse_combs(action,range_size):
    Num = action[2] - 1 + 5 * (action[1] - 1) + 25 * (action[0] - 1)

    return int(Num)

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=10000)
        self.gamma = 0.1  # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.99
        self.max_threshold = 0.8
        self.learning_rate = 0.05
        self.action_max = [0.25918, 0.25918, 0.010472]  # maximum velocities of: [rightWheel,leftWheel,prismatic]
        self.action_buckets = [5, 5, 5]  # number of buckets for [rightWheel,leftWheel,prismatic]
        self.actionsCombination = combs(self.action_buckets)
        self.model = self._build_model()

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        model = Sequential()
        model.add(Dense(self.state_size, input_dim=self.state_size, activation='relu'))
        model.add(Dense(20, activation='relu'))
        model.add(Dense(30, activation='relu'))
        model.add(Dense(30, activation='relu'))
        model.add(Dense(len(self.actionsCombination), activation='linear'))
        model.compile(loss='mse',
                      optimizer=Adam(lr=self.learning_rate))
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if numpy.random.rand() <= self.epsilon:
            action = numpy.zeros(self.action_size)
            action[0] = random.randrange(1, self.action_buckets[0] + 1)
            action[1] = random.randrange(1, self.action_buckets[1] + 1)
            action[2] = random.randrange(1, self.action_buckets[2] + 1)
        else:
            action_reward = self.model.predict(state)
            actionNum = 0
            if numpy.random.rand() < self.max_threshold:  # choosing max
                actionNum = numpy.argmax(action_reward[0])  # returns action Number
            else:  # choosing from n max
                n = 0
                max_value = action_reward[0][numpy.argmax(action_reward[0])]
                for x in action_reward[0]:
                    if x > max_value * 0.75:
                        n += 1
                if n != 0:
                    max_args = numpy.argsort(action_reward[0])[-n:]
                    actionNum = numpy.random.choice(max_args, 1)[0]
                else:
                    actionNum = numpy.argmax(action_reward[0])

                print('max_value: ', max_value, 'choosen_action:  ', action_reward[0][actionNum], 'choosen_Num:  ',
                      actionNum)
            action = self.actionsCombination[actionNum]
            action = numpy.asarray(action)

        return action

    def predict(self, state):
        return self.model.predict(state)

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        small_batch = 20
        batch_count = 0
        for state, action, reward, next_state, done in minibatch:
            if batch_count == 0:
                batch_state = []
                batch_target = []
            if batch_count < small_batch:
                target = reward
                actionNum = reverse_combs(action, self.action_buckets)
                if not done:
                    target = (reward + self.gamma * numpy.amax(self.model.predict(next_state)[0]))

                target_f = self.model.predict(state)
                target_f[0][actionNum] = target
                batch_count += 1
                batch_state.append(state)
                batch_target.append(target_f)
            if batch_count == small_batch:
                for i in range(small_batch):
                    self.model.fit(batch_state[i], batch_target[i], epochs=1, verbose=0)
                batch_count = 0

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save(name)

    # function: Changing the predicted action number (which indicate the number of bucket) into real action, 
    # which is the velocity
    def act_into_realAct(self, action_size, action):
        # velocity can be between (-action_max,action_max)
        real_action = numpy.zeros(action_size)
        for i in range(action_size):
            range_length = (2 * self.action_max[i]) / self.action_buckets[i]
            a1 = self.action_max[i] - (range_length * (self.action_buckets[i] - action[i]))
            a0 = self.action_max[i] - (range_length * (self.action_buckets[i] - (action[i] - 1)))
            real_action[i] = (a1 + a0) / 2  # the real action value is the middle point of the bucket that is chosen
        return real_action
    
    
def check_loss(state, next_state, action_num, reward, agent):
    q_sa = agent.predict(state)[0][int(action_num)]
    q_nextsa = numpy.amax(agent.predict(next_state)[0])
    loss = pow(q_sa - (reward + agent.gamma * q_nextsa), 2)
    return loss
