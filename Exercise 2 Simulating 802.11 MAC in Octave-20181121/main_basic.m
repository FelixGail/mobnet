% System parameters are based on Bianchi's paper: Performance Analysis of the IEEE 802.11 Distributed Coordination Function



clc
clear
close

%% Student ID (Only input the last 2 digits)
% Name : Felix Clemens Gail
% Matriculation No. : 2750840

%% Input parameters

% Adjustable parameters 
input.no_nodes = 4;                                              % Number of wifi-nodes 
input.tx_prob = 0.2;                                             % Transmission probability
input.packet_payload = 4000;                                     % packet size in bits
input.cw_vec = [8 16 32 64 128 256 512 1024];                    % Vector for the contention window
input.simulation_time = 50e-3;                                   % Simulation time in second

% Fixed parameters
[input, id, state] = function_input_data(input);                 % Defined the input parameteres : Do not change this file! 

input.cts_timeout = 0,0003;
input.collision_time = input.rts_time + input.sifs_time + input.cts_timeout;

%% Initial access
% Determine the time at which a node access the channel
s = 1;
input.access = rand( 1, input.no_nodes);
for n = 1 : input.no_nodes
   while input.access(n) > input.tx_prob
      s = s + 1;
      input.access(n) = rand;
   end
   input.access_slot(n) = s;
end

% Simulation events
data.event(:, id.time) = input.access_slot .* input.slot_time;   % access time
data.event(:, id.node) = 1:input.no_nodes;                       % nodes index
data.event(:, id.state) = state.rts;                             % nodes' state
data.event(:, id.cw) = input.cw_cnt;                             % nodes' contention window
data.event = sortrows(data.event, 1);                            % Sorting events such that the one with the earliest time occur first
data.event_time = data.event(1,1);                               % The time of the first event
data.event_bank = [];

%% Start: Event-based simulator
while data.event_time < input.simulation_time
   
   current_event = data.event(1, :);
   n = current_event(id.node);
   new_event = current_event;
   
   switch current_event(id.state)  
     
      case state.rts
        collision = false;
        printf("node %d at %f (->%f->%f):\n", current_event(id.node), current_event(id.time), (current_event(id.time)+input.rts_time), (current_event(id.time)+input.tx_time));
        positions = find(data.event(:, 1) <= (current_event(id.time)+input.slot_time));
        positions(1,:) = [];
        changed_events = [];
        for position = positions'
          other_node = data.event(position, :);
          data.event_bank = [data.event_bank; other_node];
          printf("\tnode %d at %f - ", other_node(id.node), other_node(id.time));
          if input.cw_cnt(other_node(id.node)) < input.max_cw
            input.cw_cnt(other_node(id.node)) += 1;
          endif
          backoff_time = randi(input.cw_vec(input.cw_cnt(other_node(id.node)))) * input.slot_time;
          collision = true;
          other_node(id.time) = current_event(id.time) + input.collision_time + input.difs_time + backoff_time;
          %other_node(id.time) = current_event(id.time) + input.tx_time + input.difs_time + backoff_time;
          printf("new time: %f - collision: %d\n", other_node(id.time), collision);
          changed_events = [changed_events; other_node];
        endfor
        data.event(positions, :) = [];
        data.event = [data.event; changed_events];
        
        if collision
          if input.cw_cnt(current_event(id.node)) < input.max_cw
            input.cw_cnt(current_event(id.node)) += 1;
          endif
          new_event(id.time) = current_event(id.time) + input.collision_time + input.difs_time + randi(input.cw_vec(input.cw_cnt(current_event(id.node)))) * input.slot_time;
          new_event(id.state) = state.rts;
        else
          new_event(id.time) = current_event(id.time) + input.rts_time;
          new_event(id.state) = state.rts_sifs;
          
          printf("\tInterception during node %d tx ( - %f):\n", current_event(id.node), (current_event(id.time)+input.tx_time))
          positions = find(data.event(:, 1) < (current_event(id.time)+input.tx_time));
          positions(1,:) = [];
          changed_events = [];
          for position = positions'
            other_node = data.event(position, :);
            to_save = other_node
            to_save(id.node) = to_save(id.node) + 4
            data.event_bank = [data.event_bank; to_save];
            printf("\t\tnode %d at %f - backoff count: %d - ", other_node(id.node), other_node(id.time), input.cw_cnt(other_node(id.node)));
            if input.cw_cnt(other_node(id.node)) == 0
              if input.cw_cnt(other_node(id.node)) < input.max_cw
                input.cw_cnt(other_node(id.node)) += 1;
              endif
              backoff_time = randi(input.cw_vec(input.cw_cnt(other_node(id.node)))) * input.slot_time;
              other_node(id.time) = current_event(id.time) + input.tx_time + input.difs_time + backoff_time;
            else
              other_node(id.time) = other_node(id.time) + input.tx_time + input.difs_time;
              printf("remaining time: %f - ", other_node(id.time)-current_event(id.time))
            endif
            printf("new time: %f\n", other_node(id.time));
            changed_events = [changed_events; other_node];
          endfor
          data.event(positions, :) = [];
          data.event = [data.event; changed_events];
        endif
        printf("\tnew event: %d at %f\n", new_event(id.state), new_event(id.time));
      
      case state.rts_sifs
        new_event(id.time) = current_event(id.time) + input.sifs_time;
        new_event(id.state) = state.cts;
      
      case state.cts
        new_event(id.time) = current_event(id.time) + input.cts_time;
        new_event(id.state) = state.cts_sifs;
      
      case state.cts_sifs
        new_event(id.time) = current_event(id.time) + input.sifs_time;
        new_event(id.state) = state.payload;
      
      case state.payload
         new_event(id.time) = current_event(id.time) + input.payload_time;
         new_event(id.state) = state.payload_sifs;
         
      case state.payload_sifs
         new_event(id.time) = current_event(id.time) + input.sifs_time;
         new_event(id.state) = state.ack;
         
      case state.ack
         new_event(id.time) = current_event(id.time) + input.ack_time;
         new_event(id.state) = state.difs;
         
      case state.difs
         backoff_time = randi ( min(input.cw_vec) ) * input.slot_time;
         input.cw_cnt(current_event(id.node)) = 0;
         % When will it access the channel again?
         node_access = 0;
         while node_access > input.tx_prob
            s = s + 1;
            node_access = rand;
         end
         new_event(id.time) = current_event(id.time) + s*input.slot_time + input.difs_time + backoff_time;
         new_event(id.state) = state.rts;
         new_event(id.cw) = 0;
         
   end
   
   data.event(1,:) = [];
   data.event = sortrows( [data.event; new_event], 1);
   data_event = data.event;
   data.event_bank = [data.event_bank; current_event];
   data.event_time = data.event(1);
   disp_event = data.event;
   
end

%% Plotting graphs
figure(1)
hold on
box on; grid on

node_event = find(data.event_bank(:, 2) == 1);
stem(data.event_bank(node_event,1), data.event_bank(node_event,3), ':b*', 'LineWidth', 1, 'MarkerSize', 6)

node_event = find(data.event_bank(:, 2) == 2);
stem(data.event_bank(node_event,1), data.event_bank(node_event,3), ':ro', 'LineWidth', 1, 'MarkerSize', 6)

node_event = find(data.event_bank(:, 2) == 3);
stem(data.event_bank(node_event,1), data.event_bank(node_event,3), ':ks', 'LineWidth', 1, 'MarkerSize', 6)

node_event = find(data.event_bank(:, 2) == 4);
stem(data.event_bank(node_event,1), data.event_bank(node_event,3), ':gp', 'LineWidth', 1, 'MarkerSize', 6)

node_event = find(data.event_bank(:, 2) == 5);
stem(data.event_bank(node_event,1), data.event_bank(node_event,3), '-bx', 'LineWidth', 1, 'MarkerSize', 6)

node_event = find(data.event_bank(:, 2) == 6);
stem(data.event_bank(node_event,1), data.event_bank(node_event,3), '-rx', 'LineWidth', 1, 'MarkerSize', 6)

node_event = find(data.event_bank(:, 2) == 7);
stem(data.event_bank(node_event,1), data.event_bank(node_event,3), '-kx', 'LineWidth', 1, 'MarkerSize', 6)

node_event = find(data.event_bank(:, 2) == 8);
stem(data.event_bank(node_event,1), data.event_bank(node_event,3), '-gx', 'LineWidth', 1, 'MarkerSize', 6)

legend("Node1", "Node2", "Node3", "Node4", "Failed RTS Node1", "Failed RTS Node2", "Failed RTS Node3", "Failed RTS Node4")
axis([0 max(data.event_bank(:,1)) + 10*input.slot_time 0 9])
xlabel('Simulation time (sec)')
ylabel('State')

ytick = [1:8];
set(gca, 'ytick', ytick);
yticklabel = ["RTS"; "RTS SIFS"; "CTS"; "CTS_SIFS"; "Payload"; "Payload SIFS"; "ACK"; "DIFS"];
set(gca, 'yticklabel', yticklabel)
































