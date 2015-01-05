
<div class="pure-u pure-u-md-4-24">
    <div class="rapp-new-menu-item">
        <div class="rapp-new-menu-icon pure-u"><h2><span class="oi oi-file"></span></h2></div>
        <div class="pure-u-3-4">
            <h2>Basic Settings</h2>
            <p>Setup the basic information of the RAPP. Chose a universal name, current version, robots supported, 
            CPU architecture, tags identifying the new RAPP and a thumbnail that will be displayed in the RAPP store.
            </p>
        </div>
    </div>
</div>
<div class="pure-u pure-u-md-6-24 rapp-new-main">

    <form class="pure-form pure-form-aligned" id="basic_info" method="post" accept-charset="UTF-8">

        <fieldset>
        
            <div class="pure-control-group">
                <p>
                    <label for="rappname">RAPP Name</label>
                    <input id="rappname" type="text" placeholder="e.g., pathfinder" size="30">
                </p>
            </div>
            
            <div class="pure-control-group">
                <p>
                    <label for="version">RAPP Version</label>
                    <select id="major_version">
                        <option value="0" selected="">0</option>
                        <option value="1">1</option>
                        <option value="2">2</option>
                        <option value="3">3</option>
                        <option value="4">4</option>
                        <option value="5">5</option>
                        <option value="6">6</option>
                        <option value="7">7</option>
                        <option value="8">8</option>
                        <option value="9">9</option>
                    </select>
                    <select id="minor_version">
                        <option value="0" selected="">0</option>
                        <option value="1">1</option>
                        <option value="2">2</option>
                        <option value="3">3</option>
                        <option value="4">4</option>
                        <option value="5">5</option>
                        <option value="6">6</option>
                        <option value="7">7</option>
                        <option value="8">8</option>
                        <option value="9">9</option>
                    </select>
                </p>
            </div>
        
            <div class="pure-control-group">
                <p>
                <label for="Language">Programming Language</label>
                <select id="Language">
                    <option selected disabled hidden value=''></option>
                    <option value="cpp">C++</option>
                    <option value="js">JavaScript</option>
                </select>
                </p>
            </div>

            <div class="pure-control-group">
                <p>
                    <table class="rapp-line-table"><tr>
                    <td><label for="cpuarch">CPU Architecture</label></td>
                    <td><input type="checkbox" class="cpuarch" value="amd64"/><a cpuarch>  amd64</a></td></tr>
                    <tr><td></td><td><input type="checkbox" class="cpuarch" value="i386"/><a cpuarch>  i386</a></td></tr>
                    <tr><td></td><td><input type="checkbox" class="cpuarch" value="arm"/><a cpuarch>  arm</a></td></tr>
                    </table>
                </p>
            </div>
            
            <div class="pure-control-group">
                <p>
                    <table class="rapp-line-table">
                    <tr><td><label for="RobotTypes">Robot platforms</label></td>
                        <td><input type="checkbox" class="robots" value="nao"/><a cpuarch>  NAO</a></td></tr>
                    <tr><td></td><td><input type="checkbox" class="robots" value="angmed"/><a cpuarch>  Ang-Med</a></td></tr>
                    </table>
                </p>
            </div>

            <div class="pure-control-group">
                <div id="thumbnail">
                    <table class="rapp-line-table">
                        <tr><td></td><td></td><td><div id="fileuploader">Thumbnail</div></td></tr>
                    </table>
                </div>
            </div>
            
            <hr>
            
            <div class="pure-controls-group">
                <button type="submit" id="process_form" class="pure-button pure-button-primary">Next</button>
            </div>
            
        </fieldset>
    </form>
</div>

<script type="text/javascript" src="/scripts/rapp_basic.js"></script>