clc
clear

max_nRuns = 10;
max_nLocations = 42;
% Importing GSPN primitive action models (navigation/mopping/vacuuming)

PNPRO_path = "homeclean_notimeouts.PNPRO";

[nGSPN, models] = ImportfromGreatSPN(PNPRO_path);

NavigationModel = models.navigation;
MoppingModel = models.mopping;
VacuumingModel = models.vacuuming;

results.GSPNR_times = zeros(max_nRuns, max_nLocations);
results.MDP_times_without_wait = zeros(max_nRuns, max_nLocations);
results.MDP_times_with_wait = zeros(max_nRuns, max_nLocations);

for n_run = 1:max_nRuns
    disp(["Run number - " n_run]);
    for nLocations = 1:max_nLocations
        % Creating overall GSPNR model using topological map and imported models
        disp(["Number of locations - " nLocations]);
        adjacency_matrix = [];
        for r_index = 1:nLocations
            row = zeros(1, nLocations);
            if ~((r_index-1)<1)
                row(r_index-1) = 1;
            end
            if ~((r_index+1)>nLocations)
                row(r_index+1) = 1;
            end
            adjacency_matrix(r_index,:) = row;
        end
        node_array = {};
        for l_index = 1:nLocations
            array = {'L', int2str(l_index)};
            node_name = strjoin(array,'');
            node_array = [node_array, node_name];
        end

        topological_map = digraph(adjacency_matrix, node_array, 'omitselfloops');
        actions_available = struct();
        plot(topological_map)
        for l_index = 1:nLocations
            l_name = node_array{l_index};
            actions_available.(l_name) = ["mopping", "vacuuming"];
        end
        robot_marking = struct();
        robot_marking.L1 = 1;
        robot_marking.(node_array{nLocations}) = 1;
        tic
        GSPNRModel = GSPNRCreationfromTopMap(topological_map, actions_available, models, robot_marking);
        GSPNR_creation_time = toc;
        
        results.GSPNR_times(n_run, nLocations) = GSPNR_creation_time;
        
        tic
        [emb_MDP, covered_marking_list, covered_state_list, covered_state_type] = GSPNRModel.toMDP_without_wait();
        MDP_creation_time_without_wait = toc;
        
        results.MDP_times_without_wait(n_run, nLocations) = MDP_creation_time_without_wait;
        
        tic
        [emb_MDP, covered_marking_list, covered_state_list, covered_state_type] = GSPNRModel.toMDP();
        MDP_creation_time_with_wait = toc;
        
        results.MDP_times_with_wait(n_run, nLocations) = MDP_creation_time_with_wait;
        
        save("increasing_locations_results_second_iteration.mat", "results");
        
    end
end